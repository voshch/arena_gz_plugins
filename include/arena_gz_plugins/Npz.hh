#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace arena_gz_plugins
{

/// One array of a stored numpy zip, little-endian, C order.
struct NpyArray
{
  std::string descr;
  std::vector<std::size_t> shape;
  std::vector<char> data;

  std::size_t Size() const
  {
    std::size_t n = 1;
    for (const auto dim : shape)
      n *= dim;
    return n;
  }

  std::vector<std::int64_t> Ints() const
  {
    std::vector<std::int64_t> out(Size());
    for (std::size_t i = 0; i < out.size(); ++i)
    {
      if (descr == "<i4")
      {
        std::int32_t v;
        std::memcpy(&v, data.data() + 4 * i, 4);
        out[i] = v;
      }
      else if (descr == "<i8")
      {
        std::memcpy(&out[i], data.data() + 8 * i, 8);
      }
      else
      {
        throw std::runtime_error("array of type '" + descr + "' is not an integer");
      }
    }
    return out;
  }

  std::vector<double> Doubles() const
  {
    if (descr != "<f8")
      throw std::runtime_error("array of type '" + descr + "' is not <f8");
    std::vector<double> out(Size());
    std::memcpy(out.data(), data.data(), 8 * out.size());
    return out;
  }

  std::vector<std::string> Strings() const
  {
    if (descr.size() < 3 || descr.compare(0, 2, "<U") != 0)
      throw std::runtime_error("array of type '" + descr + "' is not <U");
    const std::size_t width = std::stoul(descr.substr(2));
    std::vector<std::string> out(Size());
    for (std::size_t i = 0; i < out.size(); ++i)
    {
      for (std::size_t c = 0; c < width; ++c)
      {
        std::uint32_t code;
        std::memcpy(&code, data.data() + 4 * (i * width + c), 4);
        if (code == 0)
          break;
        if (code < 0x80)
        {
          out[i] += static_cast<char>(code);
        }
        else if (code < 0x800)
        {
          out[i] += static_cast<char>(0xC0 | (code >> 6));
          out[i] += static_cast<char>(0x80 | (code & 0x3F));
        }
        else if (code < 0x10000)
        {
          out[i] += static_cast<char>(0xE0 | (code >> 12));
          out[i] += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
          out[i] += static_cast<char>(0x80 | (code & 0x3F));
        }
        else
        {
          out[i] += static_cast<char>(0xF0 | (code >> 18));
          out[i] += static_cast<char>(0x80 | ((code >> 12) & 0x3F));
          out[i] += static_cast<char>(0x80 | ((code >> 6) & 0x3F));
          out[i] += static_cast<char>(0x80 | (code & 0x3F));
        }
      }
    }
    return out;
  }
};

namespace npz_detail
{

template<typename T>
T Read(const std::vector<char> & bytes, std::size_t at)
{
  if (at + sizeof(T) > bytes.size())
    throw std::runtime_error("truncated zip");
  T value;
  std::memcpy(&value, bytes.data() + at, sizeof(T));
  return value;
}

/// Text after `key` up to the next delimiter of a numpy header dict.
inline std::string HeaderValue(const std::string & header, const std::string & key)
{
  const auto at = header.find("'" + key + "':");
  if (at == std::string::npos)
    throw std::runtime_error("npy header has no '" + key + "'");
  std::size_t begin = at + key.size() + 3;
  while (begin < header.size() && header[begin] == ' ')
    ++begin;
  if (header[begin] == '(')
    return header.substr(begin, header.find(')', begin) - begin + 1);
  if (header[begin] == '\'')
    return header.substr(begin + 1, header.find('\'', begin + 1) - begin - 1);
  return header.substr(begin, header.find_first_of(",}", begin) - begin);
}

inline NpyArray ParseNpy(const char * bytes, std::size_t size)
{
  if (size < 10 || std::memcmp(bytes, "\x93NUMPY", 6) != 0)
    throw std::runtime_error("not an npy array");
  const unsigned major = static_cast<unsigned char>(bytes[6]);
  if (major != 1 && major != 2)
    throw std::runtime_error("npy version " + std::to_string(major) + " is not supported");
  std::size_t headerLen;
  std::size_t start;
  if (major == 1)
  {
    std::uint16_t len;
    std::memcpy(&len, bytes + 8, 2);
    headerLen = len;
    start = 10;
  }
  else
  {
    std::uint32_t len;
    std::memcpy(&len, bytes + 8, 4);
    headerLen = len;
    start = 12;
  }
  if (start + headerLen > size)
    throw std::runtime_error("truncated npy header");
  const std::string header(bytes + start, headerLen);

  NpyArray array;
  array.descr = HeaderValue(header, "descr");
  if (HeaderValue(header, "fortran_order") != "False")
    throw std::runtime_error("fortran order arrays are not supported");
  const std::string shape = HeaderValue(header, "shape");
  for (std::size_t at = 0; at < shape.size();)
  {
    if (shape[at] >= '0' && shape[at] <= '9')
    {
      std::size_t used = 0;
      array.shape.push_back(std::stoul(shape.substr(at), &used));
      at += used;
    }
    else
    {
      ++at;
    }
  }
  array.data.assign(bytes + start + headerLen, bytes + size);
  std::size_t item = 0;
  if (array.descr == "<i4")
    item = 4;
  else if (array.descr == "<i8" || array.descr == "<f8")
    item = 8;
  else if (array.descr.compare(0, 2, "<U") == 0)
    item = 4 * std::stoul(array.descr.substr(2));
  else
    throw std::runtime_error("array type '" + array.descr + "' is not supported");
  if (array.data.size() < item * array.Size())
    throw std::runtime_error("truncated npy data");
  return array;
}

}  // namespace npz_detail

/// Arrays of a stored (uncompressed) numpy zip by member name without `.npy`.
inline std::unordered_map<std::string, NpyArray> ReadNpz(const std::string & path)
{
  using npz_detail::Read;
  std::ifstream in(path, std::ios::binary);
  if (!in)
    throw std::runtime_error("cannot open '" + path + "'");
  const std::vector<char> bytes((std::istreambuf_iterator<char>(in)), {});

  std::size_t eocd = std::string::npos;
  for (std::size_t at = bytes.size() < 22 ? bytes.size() : bytes.size() - 22 + 1; at > 0; --at)
  {
    if (Read<std::uint32_t>(bytes, at - 1) == 0x06054b50)
    {
      eocd = at - 1;
      break;
    }
  }
  if (eocd == std::string::npos)
    throw std::runtime_error("not a zip file");
  const std::size_t count = Read<std::uint16_t>(bytes, eocd + 10);
  std::size_t entry = Read<std::uint32_t>(bytes, eocd + 16);

  std::unordered_map<std::string, NpyArray> arrays;
  for (std::size_t i = 0; i < count; ++i)
  {
    if (Read<std::uint32_t>(bytes, entry) != 0x02014b50)
      throw std::runtime_error("bad zip central directory");
    const auto method = Read<std::uint16_t>(bytes, entry + 10);
    const auto packed = Read<std::uint32_t>(bytes, entry + 20);
    const auto size = Read<std::uint32_t>(bytes, entry + 24);
    const std::size_t nameLen = Read<std::uint16_t>(bytes, entry + 28);
    const std::size_t extraLen = Read<std::uint16_t>(bytes, entry + 30);
    const std::size_t commentLen = Read<std::uint16_t>(bytes, entry + 32);
    const std::size_t local = Read<std::uint32_t>(bytes, entry + 42);
    if (entry + 46 + nameLen > bytes.size())
      throw std::runtime_error("truncated zip");
    std::string name(bytes.data() + entry + 46, nameLen);
    entry += 46 + nameLen + extraLen + commentLen;

    if (method != 0)
      throw std::runtime_error(
        "member '" + name + "' is compressed, write the file with np.savez, not np.savez_compressed");
    if (packed == 0xFFFFFFFFu || size == 0xFFFFFFFFu)
      throw std::runtime_error("zip64 member '" + name + "' is not supported");
    if (Read<std::uint32_t>(bytes, local) != 0x04034b50)
      throw std::runtime_error("bad zip local header");
    const std::size_t data =
      local + 30 + Read<std::uint16_t>(bytes, local + 26) + Read<std::uint16_t>(bytes, local + 28);
    if (data + size > bytes.size())
      throw std::runtime_error("truncated zip");
    if (name.size() > 4 && name.compare(name.size() - 4, 4, ".npy") == 0)
      name.resize(name.size() - 4);
    arrays.emplace(name, npz_detail::ParseNpy(bytes.data() + data, size));
  }
  return arrays;
}

}  // namespace arena_gz_plugins
