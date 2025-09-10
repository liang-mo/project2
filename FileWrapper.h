#pragma once

#ifndef FILE_WRAPPER_H_
#define FILE_WRAPPER_H_

#include "CommonAlgorithm.h"
#include "Uri.h"

#include <fstream>
#include <spdlog/spdlog.h>
#include <string_view>
#include <vector>

namespace Zondy
{

class FileWrapper
{
public:
  static bool GetFileBuffer(const std::string_view& uri, std::vector<std::byte>& buffer)
  {
    const std::string filePath = Common::CommonAlgorithm::Utf8ToGbk(uri.data());
    const std::unique_ptr<std::ifstream, std::function<void(std::ifstream*)>> filePtr(new std::ifstream(filePath, std::ios::in | std::ios::binary),
                                                                                      [](std::ifstream* file) {
                                                                                        if (file)
                                                                                        {
                                                                                          file->close();
                                                                                          delete file;
                                                                                          file = nullptr;
                                                                                        }
                                                                                      });
    if (!filePtr->is_open())
    {
      SPDLOG_ERROR("Failed to open file: {}", uri);
      return false;
    }

    // 获取文件大小
    filePtr->seekg(0, std::ios::end);
    const std::streampos fileSize = filePtr->tellg();
    filePtr->seekg(0, std::ios::beg);

    // 检查文件大小是否有效
    if (fileSize == -1)
    {
      SPDLOG_ERROR("Failed to get file size: {}", uri);
      return false;
    }

    // 调整缓冲区大小
    buffer.resize(static_cast<size_t>(fileSize));

    // 读取文件内容
    filePtr->read(reinterpret_cast<char*>(buffer.data()), fileSize);
    if (!filePtr)
    {
      SPDLOG_ERROR("Failed to read file: {}", uri);
      return false;
    }

    return true;
  }
};

}  // namespace Zondy

#endif
