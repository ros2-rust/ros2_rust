#include <stdbool.h>
#include <stdio.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>

bool rclrs_native_ok() { return rcl_ok(); }

int32_t rclrs_native_init() {
  // TODO(esteve): parse args
  return rcl_init(0, NULL, rcl_get_default_allocator());
}

const char *rclrs_native_get_error_string_safe() {
  return rcl_get_error_string_safe();
}

void rclrs_native_reset_error() { rcl_reset_error(); }

int32_t rclrs_native_create_node_handle(uintptr_t *node_handle,
                                        const char *name,
                                        const char *namespace) {
  rcl_node_t *node = (rcl_node_t *)malloc(sizeof(rcl_node_t));
  *node = rcl_get_zero_initialized_node();

  rcl_node_options_t default_options = rcl_node_get_default_options();
  rcl_ret_t ret = rcl_node_init(node, name, namespace, &default_options);
  *node_handle = (uintptr_t)node;
  return ret;
}

int32_t rclrs_native_create_publisher_handle(
    uintptr_t *publisher_handle, uintptr_t node_handle,
    uintptr_t type_support_handle, const char *topic,
    uint8_t qos_policy_history, size_t depth, uint8_t qos_policy_reliability,
    uint8_t qos_policy_durability, bool avoid_ros_namespace_conventions) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_message_type_support_t *type_support =
      (rosidl_message_type_support_t *)type_support_handle;

  rcl_publisher_t *publisher =
      (rcl_publisher_t *)malloc(sizeof(rcl_publisher_t));
  *publisher = rcl_get_zero_initialized_publisher();
  rcl_publisher_options_t publisher_ops = rcl_publisher_get_default_options();

  rmw_qos_profile_t qos_profile = {
      qos_policy_history, depth, qos_policy_reliability, qos_policy_durability,
      avoid_ros_namespace_conventions};

  publisher_ops.qos = qos_profile;

  rcl_ret_t ret =
      rcl_publisher_init(publisher, node, type_support, topic, &publisher_ops);
  *publisher_handle = (uintptr_t)publisher;
  return ret;
}

int32_t rclrs_native_publish(uintptr_t publisher_handle,
                             uintptr_t message_handle) {
  rcl_publisher_t *publisher = (rcl_publisher_t *)publisher_handle;

  void *raw_ros_message = (void *)message_handle;

  rcl_ret_t ret = rcl_publish(publisher, raw_ros_message);

  return ret;
}

uintptr_t rclrs_native_get_zero_initialized_wait_set() {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)malloc(sizeof(rcl_wait_set_t));
  *wait_set = rcl_get_zero_initialized_wait_set();
  return (uintptr_t)wait_set;
}

void rclrs_native_destroy_wait_set(uintptr_t wait_set_handle) {
  free((rcl_wait_set_t *)wait_set_handle);
}

int32_t rclrs_native_wait_set_init(uintptr_t wait_set_handle,
                                   size_t number_of_subscriptions,
                                   size_t number_of_guard_conditions,
                                   size_t number_of_timers,
                                   size_t number_of_clients,
                                   size_t number_of_services) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;

  rcl_ret_t ret = rcl_wait_set_init(
      wait_set, number_of_subscriptions, number_of_guard_conditions,
      number_of_timers, number_of_clients, number_of_services,
      rcl_get_default_allocator());

  return ret;
}

int32_t rclrs_native_wait_set_clear_subscriptions(uintptr_t wait_set_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait_set_clear_subscriptions(wait_set);

  return ret;
}

int32_t rclrs_native_wait_set_clear_services(uintptr_t wait_set_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait_set_clear_services(wait_set);

  return ret;
}

int32_t rclrs_native_wait_set_clear_clients(uintptr_t wait_set_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait_set_clear_clients(wait_set);

  return ret;
}

int32_t rclrs_native_wait_set_add_subscription(uintptr_t wait_set_handle,
                                               uintptr_t subscription_handle) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_subscription_t *subscription = (rcl_subscription_t *)subscription_handle;
  rcl_ret_t ret = rcl_wait_set_add_subscription(wait_set, subscription);

  return ret;
}

int32_t rclrs_native_wait(uintptr_t wait_set_handle, int64_t timeout) {
  rcl_wait_set_t *wait_set = (rcl_wait_set_t *)wait_set_handle;
  rcl_ret_t ret = rcl_wait(wait_set, timeout);

  return ret;
}

int32_t rclrs_native_create_subscription_handle(
    uintptr_t *subscription_handle, uintptr_t node_handle,
    uintptr_t type_support_handle, const char *topic,
    uint8_t qos_policy_history, size_t depth, uint8_t qos_policy_reliability,
    uint8_t qos_policy_durability, bool avoid_ros_namespace_conventions) {
  rcl_node_t *node = (rcl_node_t *)node_handle;

  rosidl_message_type_support_t *type_support =
      (rosidl_message_type_support_t *)type_support_handle;

  rcl_subscription_t *subscription =
      (rcl_subscription_t *)malloc(sizeof(rcl_subscription_t));
  *subscription = rcl_get_zero_initialized_subscription();
  rcl_subscription_options_t subscription_ops =
      rcl_subscription_get_default_options();

  rmw_qos_profile_t qos_profile = {
      qos_policy_history, depth, qos_policy_reliability, qos_policy_durability,
      avoid_ros_namespace_conventions};
  subscription_ops.qos = qos_profile;

  rcl_ret_t ret = rcl_subscription_init(subscription, node, type_support, topic,
                                        &subscription_ops);

  *subscription_handle = (uintptr_t)subscription;
  return ret;
}

int32_t rclrs_native_take(uintptr_t subscription_handle, uintptr_t message_handle) {
  rcl_subscription_t * subscription = (rcl_subscription_t *)subscription_handle;
  void * taken_msg = message_handle;

  rcl_ret_t ret = rcl_take(subscription, taken_msg, NULL);

  return ret;
}
