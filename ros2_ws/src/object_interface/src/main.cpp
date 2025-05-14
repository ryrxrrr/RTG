#include <QApplication>
#include <thread>
#include "qt_node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  auto qt_node = std::make_shared<QtNode>("my_qt_node");
  qt_node->initMoveIt();
  qt_node->show();

  std::thread spin_thread([qt_node](){
    rclcpp::spin(qt_node);
    rclcpp::shutdown();
  });

  int ret = app.exec();
  spin_thread.join();
  return ret;
}
