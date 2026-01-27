#pragma once

#include <sstream>
#include <string>
#include <vector>

namespace pal_policy_utils
{

inline std::vector<std::string>
split_string(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

inline std::vector<double>
parse_doubles(const std::string & str, char delimiter = ',')
{
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

}  // namespace pal_policy_common
