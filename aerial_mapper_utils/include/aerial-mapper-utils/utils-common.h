/*
 *    Filename: utils-common.h
 *  Created on: Oct 28, 2017
 *      Author: Timo Hinzmann
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#ifndef UTILS_COMMON_H_
#define UTILS_COMMON_H_

// SYSTEM
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

// NON-SYSTEM
#include <glog/logging.h>

namespace utils {
static constexpr int nameWidth = 30;

std::string paramToString(const std::string& name, double value);
std::string paramToString(const std::string& name, int value);
std::string paramToString(const std::string& name, bool value);
std::string paramToString(const std::string& name, const std::string& value);

template <typename Functor>
void parFor(int num_items, const Functor& functor, size_t num_threads) {
  CHECK_GT(num_threads, 0u) << "Num threads must be larger than 0.";
  std::vector<std::vector<size_t> > blocks;
  int num_items_per_block = num_items;
  num_items_per_block = std::ceil(static_cast<double>(num_items) /
                                  static_cast<double>(num_threads));
  const int num_blocks = std::ceil(static_cast<double>(num_items) /
                                   static_cast<double>(num_items_per_block));
  blocks.resize(num_blocks);

  int data_index = 0;

  std::vector<std::thread> threads;
  for (size_t block_idx = 0; block_idx < blocks.size(); ++block_idx) {
    std::vector<size_t>& block = blocks[block_idx];
    for (int item_idx = 0;
         (item_idx < num_items_per_block) && (data_index < num_items);
         ++item_idx) {
      block.push_back(data_index);
      ++data_index;
    }
    threads.push_back(
        std::thread([&functor, &block]() -> void { functor(block); }));
  }

  CHECK_EQ(threads.size(), blocks.size());
  for (size_t block_idx = 0; block_idx < blocks.size(); ++block_idx) {
    threads[block_idx].join();
  }
}

}  // namespace utils

#endif  // COMMON_H_
