/**
 * @file id_queue.h
 * @author Yijun Mao
 * @brief
 * @version 0.1
 * @date 2022-3-11
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_ID_QUEUE_H_
#define _CORE_COMMON_INC_BASICS_ID_QUEUE_H_


#include <assert.h>

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <queue>

#include "common/basics/basics.h"

namespace common {

/**
 * @brief 存储当前需要做行为规划的vehicle的Id
 * 内部使用一个队列，队列保存当前需要做行为规划的车辆的Id，当有变动时，需要新写入ids，当完成一个车辆的规划时，需要pop出最上层的id
 * TODO for MLAD: 单例模式，读写锁，条件锁？
 */
class PriorityIdQueue {
public:
  static PriorityIdQueue* getInstance() {
    if (instance==nullptr) {
      // 双重锁定
      std::lock_guard<std::mutex> lock_guard(init_mtx); 
      // 在std::lock_guard对象构造时，传入的mutex对象(即它所管理的mutex对象)会被当前线程锁住。在lock_guard对象被析构时，它所管理的mutex对象会自动解锁，不需要程序员手动调用lock和unlock对mutex进行上锁和解锁操作
      if (instance==nullptr) {
        instance = new PriorityIdQueue();
      }
    }
    return instance;
  }

  bool Empty();

  int Size();

  /**
   * @brief 返回此时优先级最高的车辆的id，同时pop它。若此时queue是空的，则返回-1
   */
  int Front();

  /**
   * @brief 检查是否更新队列了。线程开头只能使用Front()，通过是否是-1来判断是否为空。结尾等待agv回传结果期间，需要调用此函数，检查是否有新的update，若有，则停止等待回传结果
   */
  bool CheckUpdate();

  /**
   * @brief 在计算出新的优先级后，更新现有优先级队列。传入为一个数组，优先级高的排在前面
   */
  ErrorType UpdateQueue(const std::vector<int> &PriorityIds);

private:
  //禁止在外部初始化，复制和拷贝
  PriorityIdQueue() { std::cout << "construct PriorityIdQueue in Singleton mode" << std::endl;};
  PriorityIdQueue(const PriorityIdQueue & ) = delete;
  PriorityIdQueue & operator=(const PriorityIdQueue & ) = delete;
  static PriorityIdQueue *instance;
  static std::mutex init_mtx; // 用于懒汉模式初始化时的锁

  std::queue<int> id_queue;
  static std::mutex queue_mtx; // 用于读、修改队列时的锁
  bool update_flag=false; // 
};


}

#endif