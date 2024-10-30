#include <ui/calib_ui.h>

using namespace licalib;

int main(int argc, char **argv) {
    ros::init(argc, argv, "li_calib_gui");
    ros::NodeHandle n("~");

    CalibInterface calib_app(n);

    calib_app.renderingLoop();
    return 0;
}
