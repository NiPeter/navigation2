#ifndef BT_ZMQ_PUBLISHER_H
#define BT_ZMQ_PUBLISHER_H

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include <array>
#include <future>

namespace nav2_behavior_tree {
class PublisherZMQ : public BT::StatusChangeLogger {

public:
  PublisherZMQ(const BT::Tree &tree, unsigned max_msg_per_second = 25,
               unsigned publisher_port = 1666, unsigned server_port = 1667);

  virtual ~PublisherZMQ();

private:
  virtual void callback(BT::Duration timestamp, const BT::TreeNode &node,
                        BT::NodeStatus prev_status,
                        BT::NodeStatus status) override;

  virtual void flush() override;

  const BT::Tree &tree_;
  std::vector<uint8_t> tree_buffer_;
  std::vector<uint8_t> status_buffer_;
  std::vector<BT::SerializedTransition> transition_buffer_;
  std::chrono::microseconds min_time_between_msgs_;

  std::atomic_bool active_server_;
  std::thread thread_;

  void createStatusBuffer();

  BT::TimePoint deadline_;
  std::mutex mutex_;
  std::atomic_bool send_pending_;

  std::future<void> send_future_;

  struct Pimpl;
  Pimpl *zmq_;
};
} // namespace nav2_behavior_tree

#endif
