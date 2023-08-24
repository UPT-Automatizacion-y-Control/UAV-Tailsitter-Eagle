#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    delay(100);
  }
}
rcl_node_t node;
rcl_publisher_t publisher_attitude;
rcl_publisher_t publisher_attitude_references;
geometry_msgs__msg__Twist attitude;
geometry_msgs__msg__Twist attitude_references;
rclc_executor_t executor_pub;
rclc_support_t support;
rcl_allocator_t allocator;

void uRos_Conf(){
  // Configuración de uRos
  allocator = rcl_get_default_allocator();

  // Inicialización del nodo de uRos
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  node = rcl_get_zero_initialized_node();
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  
  // Creaión del Nodo
  RCCHECK(rclc_node_init_with_options(&node, "uav_esp32", "", &support, &node_ops));

  // Publisher para enviar la actitud actual de UAV
  RCCHECK(rclc_publisher_init_best_effort(
  &publisher_attitude,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"attitude"));

    // Publisher para enviar las referencias que se están recibiendo con el Joystick
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher_attitude_references,&node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs,msg,Twist),"attitude_references"));

  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 2, &allocator));

}