#include "common/basics/id_queue.h"

namespace common {

PriorityIdQueue* PriorityIdQueue::instance = nullptr; //static类成员必须类外定义
std::mutex PriorityIdQueue::init_mtx; //static类成员必须类外定义
std::mutex PriorityIdQueue::queue_mtx;

bool PriorityIdQueue::Empty() {
	std::lock_guard<std::mutex> lock_guard(queue_mtx);
	// printf("[PriorityIdQueue::Empty] query is empty \n");
	return id_queue.empty();
}

int PriorityIdQueue::Front() {
	std::lock_guard<std::mutex> lock_guard(queue_mtx);
	printf("[PriorityIdQueue::Front] get front id \n");
	if (id_queue.empty()) return -1;

	if (update_flag) update_flag=false;
	
	int target_id = id_queue.front();
	id_queue.pop();
	printf("[PriorityIdQueue::Front] left %d\n", id_queue.size());
	return target_id;
}

int PriorityIdQueue::Size() {
	std::lock_guard<std::mutex> lock_guard(queue_mtx);
	return id_queue.size();
}

bool PriorityIdQueue::CheckUpdate() {
	return update_flag;
}

ErrorType PriorityIdQueue::UpdateQueue(const std::vector<int> &PriorityIds) {
	std::lock_guard<std::mutex> lock_guard(queue_mtx);

	std::queue<int> empty;
	std::swap(empty, id_queue);
	// for (const auto &id:PriorityIds) {
	// 	id_queue.push(id);
	// 	printf("[PriorityIdQueue::UpdateQueue] id: %d\n", id);
	// }
	for(int i = 1; i <= 8; i++){
		id_queue.push(i);
	}

	update_flag=true;

	return kSuccess;
}

}