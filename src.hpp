// Problem 065 - A Naïve but Block-based Allocator
// Header-only implementation of a simple block-based int allocator.

#pragma once

#include <cstddef>
#include <vector>
#include <unordered_map>
#include <algorithm>

// Forward declarations. The judge will provide definitions in the driver.
int* getNewBlock(int n);
void freeBlock(const int* block, int n);

class Allocator {
public:
    Allocator() = default;

    ~Allocator() {
        // Free all active blocks
        for (auto& blk : blocks_) {
            if (blk.base) {
                freeBlock(blk.base, blk.units);
            }
        }
        // Free all reusable (empty) blocks kept in pool
        for (auto& entry : free_pool_) {
            for (auto* blk : entry.second) {
                if (blk && blk->base) {
                    freeBlock(blk->base, blk->units);
                }
                delete blk;
            }
        }
        blocks_.clear();
        alloc_index_.clear();
        free_pool_.clear();
    }

    // Allocate a sequence of memory space of n int.
    int* allocate(int n) {
        if (n <= 0) return nullptr;

        // Try to use the last acquired active block if it has enough contiguous tail
        if (!blocks_.empty()) {
            Block& last = blocks_.back();
            if (last.capacity - last.used >= n) {
                return allocate_from_block(last, n);
            }
        }

        // Otherwise try to reuse an empty block from the pool (no new OS allocation)
        const int required_units = units_for_ints(n);
        Block* reused = acquire_from_pool(required_units, n);
        if (reused) {
            blocks_.push_back(std::move(*reused));
            delete reused; // the content has been moved into blocks_.back()
            Block& last = blocks_.back();
            return allocate_from_block(last, n);
        }

        // Need to get a new block from the system
        int units = required_units;
        int* base = getNewBlock(units);
        Block fresh;
        fresh.base = base;
        fresh.units = units;
        fresh.capacity = units * (BLOCK_BYTES / static_cast<int>(sizeof(int)));
        fresh.used = 0;
        blocks_.push_back(std::move(fresh));
        Block& last = blocks_.back();
        return allocate_from_block(last, n);
    }

    // Deallocate the memory that is allocated by allocate.
    void deallocate(int* pointer, int /*n*/) {
        if (!pointer) return;
        auto it = alloc_index_.find(pointer);
        if (it == alloc_index_.end()) return; // undefined behaviour; ignore safely

        Block* blk = it->second.block;
        int idx = it->second.index_in_block;
        // Mark as freed
        if (blk->alloc_alive[idx]) {
            blk->alloc_alive[idx] = false;
            blk->live_count--;
        }
        alloc_index_.erase(it);

        // If this deallocation is at the top of the block's stack, shrink used
        while (!blk->alloc_sizes.empty() && !blk->alloc_alive.back()) {
            int sz = blk->alloc_sizes.back();
            blk->alloc_sizes.pop_back();
            blk->alloc_alive.pop_back();
            blk->used -= sz;
        }

        // If block becomes empty, move it to the reusable pool or free it
        if (blk->live_count == 0) {
            // Find the block in active list and move it to pool as reusable
            for (size_t i = 0; i < blocks_.size(); ++i) {
                if (&blocks_[i] == blk) {
                    // Move a copy into heap-allocated Block for pool storage
                    Block* empty_copy = new Block(blocks_[i]);
                    // Reset metadata for clean reuse
                    empty_copy->used = 0;
                    empty_copy->alloc_sizes.clear();
                    empty_copy->alloc_alive.clear();
                    empty_copy->live_count = 0;

                    // Erase from active blocks by swapping with back
                    if (i + 1 != blocks_.size()) std::swap(blocks_[i], blocks_.back());
                    blocks_.pop_back();

                    // Put into free pool keyed by units
                    free_pool_[empty_copy->units].push_back(empty_copy);
                    break;
                }
            }
        }
    }

private:
    static constexpr int BLOCK_BYTES = 4096;

    struct Block {
        int* base = nullptr;     // base pointer from getNewBlock
        int capacity = 0;        // capacity in ints
        int used = 0;            // used in ints (monotonic bump within the block)
        int units = 0;           // number of 4096-byte units used to obtain this block
        int live_count = 0;      // number of live allocations inside this block
        std::vector<int> alloc_sizes;  // LIFO stack of allocation sizes
        std::vector<bool> alloc_alive; // whether the corresponding allocation is alive
    };

    struct AllocRecord {
        Block* block;
        int index_in_block;
    };

    // Active blocks in the order they were acquired (last is the most recent)
    std::vector<Block> blocks_;
    // Map from returned pointer to its allocation record
    std::unordered_map<int*, AllocRecord> alloc_index_;
    // Pool of empty reusable blocks keyed by units size
    std::unordered_map<int, std::vector<Block*>> free_pool_;

    static int units_for_ints(int n_ints) {
        const std::size_t bytes = static_cast<std::size_t>(n_ints) * sizeof(int);
        int units = static_cast<int>((bytes + BLOCK_BYTES - 1) / BLOCK_BYTES);
        if (units <= 0) units = 1;
        return units;
    }

    int* allocate_from_block(Block& blk, int n) {
        int* ptr = blk.base + blk.used;
        blk.used += n;
        blk.alloc_sizes.push_back(n);
        blk.alloc_alive.push_back(true);
        blk.live_count++;
        alloc_index_[ptr] = AllocRecord{&blk, static_cast<int>(blk.alloc_sizes.size() - 1)};
        return ptr;
    }

    // Acquire an empty block from pool that can fit at least n ints. Prefer smallest units that fits.
    Block* acquire_from_pool(int required_units, int n_ints) {
        // First try exact units, then increasing
        Block* chosen = nullptr;
        int chosen_units = 0;
        // Gather candidate unit sizes
        std::vector<int> keys;
        keys.reserve(free_pool_.size());
        for (const auto& kv : free_pool_) keys.push_back(kv.first);
        std::sort(keys.begin(), keys.end());
        const int needed_units = std::max(required_units, 1);
        for (int units : keys) {
            if (units < needed_units) continue;
            auto& vec = free_pool_[units];
            if (!vec.empty()) {
                chosen = vec.back();
                vec.pop_back();
                chosen_units = units;
                break;
            }
        }
        if (!chosen) return nullptr;
        // Ensure capacity is correctly set
        chosen->units = chosen_units;
        chosen->capacity = chosen_units * (BLOCK_BYTES / static_cast<int>(sizeof(int)));
        chosen->used = 0;
        chosen->live_count = 0;
        chosen->alloc_sizes.clear();
        chosen->alloc_alive.clear();
        return chosen;
    }
};

