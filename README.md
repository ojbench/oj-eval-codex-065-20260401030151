# Problem 065 - A Naïve but Block-based Allocator

**ACMOJ Problem ID**: 1766

## Table of Contents

- [Problem 065 - A Naïve but Block-based Allocator](#problem-065-a-naïve-but-block-based-allocator)
  - [Table of Contents](#table-of-contents)
  - [Introduction](#introduction)
    - [Background](#background)
  - [Assignment Description](#assignment-description)
    - [Grade Composition](#grade-composition)
  - [Assignment Requirements](#assignment-requirements)
    - [Input Format](#input-format)
    - [Output Format](#output-format)
    - [Samples](#samples)
    - [Data Constraints](#data-constraints)
  - [Per-Testcase Resource Limits](#per-testcase-resource-limits)
  - [Test Data](#test-data)
  - [Submission Requirements](#submission-requirements)
    - [Submission Format](#submission-format)
    - [Submission Guidelines](#submission-guidelines)
    - [Evaluation Notes](#evaluation-notes)
    - [Academic Integrity](#academic-integrity)

## Introduction

### Background

Implement block-based memory allocator (4096-byte blocks) as STL allocator. Template class.

## Assignment Description

### Grade Composition

| Grading Component | Percentage |
| :--: | :--: |
| Pass **1766. A Naïve but Block-based Allocator** | 80% |
| Code Review | 20% |

Here are several points that need clarification:

- In the Code Review, we will **strictly examine your code style and repository organization structure, etc.**

- This assignment provides some sample data for testing, stored in the `/workspace/data/065/data_test/` directory. Note that these are not the test cases on the Online Judge. Passing all local test cases does not guarantee that you will pass the OJ tests.

- Besides the provided sample data, we also encourage you to design your own test data based on your program logic to assist debugging.

## Assignment Requirements

### Problem Description

<style>
p, pre {
    margin: 0.5rem 0;
}
</style>

#### 题目背景

<!-- 我们在 ICPC 大作业里已经熟悉了 STL 的使用，所以助教想给你们介绍一些 STL 进阶知识。-->

在STL中，allocator是一个你很可能没听说但是你一直在使用的工具。STL中到处是allocator的身影，如在`std::vector`中，

```c++
template<
    class T,
    class Allocator = std::allocator<T>
> class vector;
```

其默认模板参数是 `std::allocator`。

不过，最常用的未必是性能最好的。针对不同需求，我们会有不同的分配方式。有时，我们可能需要尽可能把内存分配地更紧密一些——而不是使用如 `new` 的接口。

#### 题目描述

本题中，我们要求你实现一个基于 4096 Bytes 倍数的块分配内存的 int Allocator。

**注意**：本题中，4096 Bytes 是最小的分配单元；「**块**」指的是 4096 Bytes 的倍数，即 $4096\times n$ Bytes。

简单地说，你需要调用以下函数来获得可分配的内存空间（只需要保证分配出的内存是从 `getNewBlock` 获得的即可）。这个函数可以给你 $4096\times n$ Bytes 的连续内存地址。

```c++
int* getNewBlock(int n);
```

为了避免内存泄漏，我们提供以下接口来释放分配的块。请注意，`n` 必须与调用 `getNewBlock` 时的 `n` 相同。

```c++
void freeBlock(const int* block, int n);
```

**请注意**，你可以使用 STL，但分配出的空间**不允许是手动使用 `new` 和 `delete` 申请的**（你可以使用 `new` 和 `delete` 在类中申请内存，用以存储内部状态，但不能把 `new` 得到的内存分配出去，即不能 `return new int[n]` 及其等价操作，否则你可能会得到 Wrong Answer 的评测结果）。

你需要实现一个名为 `Allocator` 的类，其对外有两个接口，分别分配**连续的 `n` 个 `int`** 和回收分配的空间。类构造时，不能先申请很多块，块必须**按需申请**；当类析构时，你**必须**释放所有的空间（**包括没有被 `deallocate` 的空间**，因为调用者可能会忘了释放内存）。（后面有代码可供使用的代码模板）

<details>
<summary><strong>点击展开 Allocator 的详细作用</strong></summary>

Allocator 是对于内存分配器的广泛称法。任何的 allocator 都至少有两个功能——内存分配（类似于 `new`）和内存解分配（类似于 `delete`）。当外界调用内存分配的时候（我们通常将调用的成员函数称为 `allocate`），allocator 将会从自己的资源空间中划分出一个部分，将这部分空间以指针的形式给到调用者。同理，当外界调用内存解分配的时候（我们通常将调用的成员函数称为 `deallocate`），allocator 可以重新分配这块空间。

</details>

```c++
int* Allocator::allocate(int n);
void Allocator::deallocate(int* pointer, int n);
```

<details>
<summary><strong>如果你还是不理解什么是 <code>Allocator::allocate</code> 和 <code>Allocator::deallocate</code>，请点此展开</strong></summary>

更通俗的讲，`Allocator` 类可以获得 $4096\times n$ Bytes 的连续空间。`int* Allocator::allocate(int n);` 需要你利用 `getNewBlock` 获得的空间进行类似 `new int[n]`；`void Allocator::deallocate(int* pointer, int n);` 需要你回收内存，类似于 `delete[] pointer`。
</details>

我们保证调用时 $n > 0$，`deallocate` 函数输入的 `n` 与调用 `allocate` 时的 `n` 统一。

#### 空间回收要求

为了避免大量申请内存但使用率极低的情况，我们要求：（我们不要求利用率 100%）
- 如果上一次获得的新块仍有足够的空间（不考虑 allocate 后再被 deallocate 的部分），则不能再获取新块。

  具体来说，你需要保证
```c++
for (int i = 0; i < 5000; ++i) {
    int* a = allocator.allocate(1);
    *a = i;
}
```
  此时不能分配 5000 块空间，必须把一部分数据合在同一个数据块上。
- 如果一块（块是 4096 Bytes 的倍数）里不含有被使用的数据，则该块必须要么可被重新分配，要么调用 `freeBlock` 以释放内存。
- 如果最后 allocate 的空间被 deallocate，则该空间仍然可以分配。

  具体来说，
```c++
int* a = allocator.allocate(50);
int* b = allocator.allocate(2000);
allocator.deallocate(b, 2000);
b = allocator.allocate(2000); // 会利用先前分配的空间
allocator.deallocate(b, 2000);
```

你可以使用以下模板：
```c++
/**
 * Get specified 4096 * n bytes from the memory.
 * @param n
 * @return the address of the block
 */
int* getNewBlock(int n);

/**
 * Free specified 4096 * n bytes from the memory.
 * @param block the pointer to the block
 * @param n
 */
void freeBlock(const int* block, int n);

// include whatever you want

class Allocator {
public:
    Allocator() {
        // TODO： constructor 构造函数
    }

    ~Allocator() {
        // TODO： destructor 析构函数
        // Please note that even if some of data is not deallocated explicitly,
        // you should free them anyway.
        // 注意，即使此时仍然有没有被 deallocate 的数据，你也要将这些内存释放。
    }

    /**
     * Allocate a sequence of memory space of n int.
     * @param n
     * @return the pointer to the allocated memory space
     */
    int* allocate(int n) {
        // TODO： allocate member function
        // You should use the int* getNewBlock(int n) function
        // 请使用 int* getNewBlock(int n) 函数来获得可向外分配的内存
        // DO NOT return the memory resource that is got from new or malloc.
        // 请不要返回从 new 或者 malloc 获得的内存地址
    }

    /**
     * Deallocate the memory that is allocated by the allocate member
     * function. If n is not the number that is called when allocating,
     * the behaviour is undefined.
     */
    void deallocate(int* pointer, int n) {
        // TODO： deallocate the pointer
    }

private:
    // Add whatever you want to add
};
```

### 测试数据

```c++
#include "src.hpp"

#include <iostream>

int usedBlocks = 0;
int usedSpace = 0;
int maxUsedSpace = 0;

int* getNewBlock(int n) {
    if (n <= 0) return nullptr;
    usedBlocks += n;
    return new int[n * 4096 / sizeof(int)];
}

void freeBlock(const int* block, int n) {
    if (block == nullptr || n <= 0) return;
    delete[] block;
}

int* allocate(Allocator& allocator, int n) {
    usedSpace += n;
    if (usedSpace > maxUsedSpace) maxUsedSpace = usedSpace;
    return allocator.allocate(n);
}

void deallocate(Allocator& allocator, int* pointer, int n) {
    usedSpace -= n;
    allocator.deallocate(pointer, n);
}

bool check(Allocator& allocator) {
    return (usedBlocks - 1) * 4096 / sizeof(int) / 2 < maxUsedSpace;
}

int main() {
    Allocator allocator;
    int* a = allocate(allocator, 100);
    for (int i = 0; i < 100; ++i) {
        a[i] = i;
    }
    int* b = allocate(allocator, 40960 / sizeof(int));
    for (int i = 0; i < 40960 / sizeof(int); ++i) {
        b[i] = i;
    }
    deallocate(allocator, b, 40960 / sizeof(int));
    b = allocate(allocator, 4096 / sizeof(int)); // will not allocate a new block
    for (int i = 0; i < 4096 / sizeof(int); ++i) {
        b[i] = i;
    }
    deallocate(allocator, b, 4096 / sizeof(int));
    b = allocate(allocator, 4096 / sizeof(int));
    for (int i = 0; i < 4096 / sizeof(int); ++i) {
        b[i] = i;
    }
    for (int i = 0; i < 100; ++i) {
        if (a[i] != i) {
            std::cout << "Error: Data integrity is not okay!" << std::endl;
            return 0;
        }
    }
    for (int i = 0; i < 4096 / sizeof(int); ++i) {
        if (b[i] != i) {
            std::cout << "Error: Data integrity is not okay!" << std::endl;
            return 0;
        }
    }
    if (!check(allocator)) {
        std::cout << "Using too much space" << std::endl;
    } else {
        std::cout << "Passed" << std::endl;
    }
}
```

### Input Format

See the problem description above.

### Output Format

See the problem description above.

### Samples

No sample data provided for this problem.

### Data Constraints

保证 `allocate` 函数调用次数小于 $1000$ 次，并保证 `deallcate` 函数调用时的 `n` 与 `allocate` 时一致。保证不会调用 `Allocator` 的拷贝构造函数和移动构造函数。

此外，
- 对于 $10\\%$ 的数据，只调用 `allocate`，不调用 `deallocate`。
- 对于 $40\\%$ 的数据，不检查空间占用。

数据点考察点
1. 仅 allocate（无 deallocate），数据完整性
2. allocate & deallocate，无占用率检查
3. allocate & deallocate，无占用率检查
4. allocate & deallocate，多个小数据请求，要求一个块上可以不止存一个数据
5. allocate & deallocate，对于最后一次 allocate 出的数据，则此时 deallocate 后必须保证空间一定可以利用
6. allocate & deallocate，大小数据块，其中大数据块大于 4096 Bytes
7. allocate & deallocate，空块必须可重新分配
8. 综合测试
9. 综合测试
10. 综合测试

## Per-Testcase Resource Limits

- **Time Limit (per test case)**: 10000 ms
- **Memory Limit (per test case)**: 256 MiB
- **Disk Usage**: No disk usage is permitted.

## Test Data

The test data for this problem is located at `/workspace/data/065/data_test/`.

## Submission Requirements

### Submission Format

Submit a single C++ source file to the Online Judge. The OJ provides its own `main.cpp` which `#include "src.hpp"`. Your submission will be written to `src.hpp` and compiled together with the OJ's driver code.

### Submission Guidelines

- The submitted code must be able to compile successfully through the above compilation process
- The program needs to be able to read data from standard input and write results to standard output
- Please ensure the code runs correctly within the given time and space limits
- **You must use C++ or C language** to implement this assignment

### Evaluation Notes

- The evaluation system will test your program using the provided test data
- The program output must exactly match the expected output (including format)
- Exceeding time or memory limits will be judged as the corresponding error type
- Please pay attention to the overall time performance of your code and the time complexity of each part of your algorithm.

### Academic Integrity

If any violations are found during evaluation or code review (including but not limited to using unconventional methods to pass test cases), your final score may be significantly reduced or become **0 points**.
