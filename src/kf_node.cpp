#include "kf_node.h"
#include "xkalmanfilterkernel.h"

KFNode::KFNode(const std::string & node_name, const std::string & node_namespace) : rclcpp::Node(node_name, node_namespace), bram_in(0, 0x2000), bram_out(1, 0x2000) {

  // Custom code here to initialize BRAM and xkalmanfilterkernel
  // ...
  InstancePtr = new XKalmanfilterkernel;
  InstancePtr->Axi_cpu_BaseAddress = 0xa0020000;
  InstancePtr->IsReady = false;
  const char* InstanceName = "KalmanFilterKernel_0";
  assert(XKalmanfilterkernel_Initialize(InstancePtr ,InstanceName) == XST_SUCCESS);
  XKalmanfilterkernel_Set_q(InstancePtr, 1);
  XKalmanfilterkernel_Set_r(InstancePtr, 1);

  // Initialize subscribers
  pos_meas_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/sensor/pos_measurement", 
    10, 
    std::bind(&KFNode::pos_meas_callback, this, std::placeholders::_1)
  );
  control_input_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
    "/controller/control_input", 
    10, 
    std::bind(&KFNode::control_input_callback, this, std::placeholders::_1)
  );

  // Initialize publishers
  pos_est_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/kf/pos_est", 10);

}

KFNode::~KFNode() {
  // Custom code here to close BRAM and xkalmanfilterkernel
  // ...
  XKalmanfilterkernel_Release(InstancePtr);
  delete InstancePtr;
}


void KFNode::call_kalman_filter_if_both_queues_not_empty() {

  if (!pos_meas_queue.empty() && !control_input_queue.empty()) {
    while (!XKalmanfilterkernel_IsIdle(InstancePtr));
    pos_t a = pos_meas_queue.front();
    acc_t b = control_input_queue.front();    
    pos_meas_queue.pop();
    control_input_queue.pop();    

    bram_in[0] = a.x;
    bram_in[1] = a.y;
    bram_in[2] = a.z;
    bram_in[3] = b.ax;
    bram_in[4] = b.ay;
    bram_in[5] = b.az;

    XKalmanfilterkernel_Start(InstancePtr);
  }
}

void KFNode::pos_meas_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  pos_t pos_meas;
  pos_meas.x = msg->data[0];
  pos_meas.y = msg->data[1];
  pos_meas.z = msg->data[2];
  pos_meas_queue.push(pos_meas);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
  call_kalman_filter_if_both_queues_not_empty();

}

void KFNode::control_input_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
  acc_t control_input;
  control_input.ax = msg->data[0];
  control_input.ay = msg->data[1];
  control_input.az = msg->data[2];
  control_input_queue.push(control_input);

  // Custom code here to possibly call Kalman filter if both queues are not empty
  // ...
  call_kalman_filter_if_both_queues_not_empty();
}

void KFNode::publish_pos_est(pos_t pos_est) {
  geometry_msgs::msg::PoseStamped pos_est_msg;
  pos_est_msg.header.stamp = this->get_clock()->now();
  pos_est_msg.header.frame_id = "world";
  pos_est_msg.pose.orientation.w = 1.0;
  pos_est_msg.pose.orientation.x = 0.;
  pos_est_msg.pose.orientation.y = 0.;
  pos_est_msg.pose.orientation.z = 0.;
  pos_est_msg.pose.position.x = pos_est.x;
  pos_est_msg.pose.position.y = pos_est.y;
  pos_est_msg.pose.position.z = pos_est.z;
  pos_est_pub_->publish(pos_est_msg);
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
    auto node = std::make_shared<KFNode>();

    rclcpp::spin(node);
  std::cout << "hej" << std::endl;

	rclcpp::shutdown();
  return 0;
}
