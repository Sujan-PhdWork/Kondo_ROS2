#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <thread>
#include <chrono>
#include <cstdio>
#include <cstring>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

// MuJoCo globals
mjModel* m = nullptr;
mjData* d = nullptr;
mjvScene scn;
mjvCamera cam;
mjvOption opt;
mjrContext* con = nullptr;
GLFWwindow* window = nullptr;

// Flags
bool running = true;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// ROS 2 globals
std::shared_ptr<rclcpp::Node> ros_node;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pos_publisher;
std::mutex sim_mutex;

// Simulation thread function
void simulate() {
    rclcpp::Rate rate(100);  // 100 Hz

    while (rclcpp::ok() && running) {
        // {
        //   std::lock_guard<std::mutex> lock(sim_mutex);
        //   mj_step(m, d);
        // }
        // Publish base x position
        auto msg = std_msgs::msg::Float64();
        {
        std::lock_guard<std::mutex> lock(sim_mutex);
        msg.data = d->qpos[0];
        }
        pos_publisher->publish(msg);

        rate.sleep();
    }
}

// // keyboard callback
// void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
//   // backspace: reset simulation
//   if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
//     mj_resetData(m, d);
//     mj_forward(m, d);
//   }
// }


// // mouse button callback
// void mouse_button(GLFWwindow* window, int button, int act, int mods) {
//   // update button state
//   button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
//   button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
//   button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

//   // update mouse position
//   glfwGetCursorPos(window, &lastx, &lasty);
// }


// // mouse move callback
// void mouse_move(GLFWwindow* window, double xpos, double ypos) {
//   // no buttons down: nothing to do
//   if (!button_left && !button_middle && !button_right) {
//     return;
//   }

//   // compute mouse displacement, save
//   double dx = xpos - lastx;
//   double dy = ypos - lasty;
//   lastx = xpos;
//   lasty = ypos;

//   // get current window size
//   int width, height;
//   glfwGetWindowSize(window, &width, &height);

//   // get shift key state
//   bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
//                     glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

//   // determine action based on mouse button
//   mjtMouse action;
//   if (button_right) {
//     action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
//   } else if (button_left) {
//     action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
//   } else {
//     action = mjMOUSE_ZOOM;
//   }

//   // move camera
//   mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
// }


// // scroll callback
// void scroll(GLFWwindow* window, double xoffset, double yoffset) {
//   // emulate vertical mouse motion = 5% of window height
//   mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
// }

void myController(const mjModel* m, mjData* d)
{
  d->ctrl[0] = std::sin(d->time);
}


// Rendering (main thread)
void renderLoop() {
  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
    return;
  }

  //create window, make OpenGL context current, request v-sync
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjrContext con;
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  // glfwSetKeyCallback(window, keyboard);
  // glfwSetCursorPosCallback(window, mouse_move);
  // glfwSetMouseButtonCallback(window, mouse_button);
  // glfwSetScrollCallback(window, scroll);
  
  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = d->time;
    {
    std::lock_guard<std::mutex> lock(sim_mutex);
    while (d->time - simstart < 1.0/60.0) {
      mj_step(m, d);
    }
    }
    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  glfwDestroyWindow(window);
  glfwTerminate();
  running = false;
}

int main(int argc, char** argv) {
    // Initialize ROS
    rclcpp::init(argc, argv);
    ros_node = std::make_shared<rclcpp::Node>("mujoco_sim_node");
    pos_publisher = ros_node->create_publisher<std_msgs::msg::Float64>("base_x_pos", 10);

    // Load model
    char error[1000] = "Could not load model";
    m = mj_loadXML("/media/sujan/Work_DIsk/one_week/mujoco_ws/src/mujoco_pubsub/Thirdparty/mujoco/model/car/car.xml", nullptr, error, 1000);
    if (!m) {
        printf("Load model error: %s\n", error);
        return 1;
    }

    d = mj_makeData(m);
    mjcb_control=myController;

    // Start simulation thread
    std::thread simThread(simulate);

    
    //This can be used when you do the subscription
    // std::thread rosSpinThread([]() {
    //     rclcpp::spin(ros_node);
    // });

    // Run rendering in main thread
    renderLoop();

    // Join threads
    simThread.join();
    // rosSpinThread.join();

    // Cleanup
    mj_deleteData(d);
    mj_deleteModel(m);
    rclcpp::shutdown();

    return 0;
}
