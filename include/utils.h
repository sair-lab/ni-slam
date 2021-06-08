#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sys/types.h>    
#include <sys/stat.h>
#include <functional>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <Eigen/Core>
#include <Eigen/StdVector>


template <template <typename, typename> class Container, typename Type>
using Aligned = Container<Type, Eigen::aligned_allocator<Type>>;

template <typename KeyType, typename ValueType>
using AlignedMap =
    std::map<KeyType, ValueType, std::less<KeyType>,
             Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename KeyType, typename ValueType>
using AlignedUnorderedMap = std::unordered_map<
    KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename KeyType, typename ValueType>
using AlignedUnorderedMultimap = std::unordered_multimap<
    KeyType, ValueType, std::hash<KeyType>, std::equal_to<KeyType>,
    Eigen::aligned_allocator<std::pair<const KeyType, ValueType>>>;

template <typename Type>
using AlignedUnorderedSet =
    std::unordered_set<Type, std::hash<Type>, std::equal_to<Type>,
                       Eigen::aligned_allocator<Type>>;

bool FileExists(const std::string& file);
bool PathExists(const std::string& path);
void ConcatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name,
    std::string* path);

std::string ConcatenateFolderAndFileName(
    const std::string& folder, const std::string& file_name);

void ReadTxt(const std::string& file_path, 
    std::vector<std::vector<std::string> >& lines, std::string seq);

#endif  // UTILS_H_