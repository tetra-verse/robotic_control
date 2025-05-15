// #include <iostream>

// #include "log/logging.h"
// #include "nlohmann/json.hpp"

// int main(int argc, char* argv[])
// {
//     Logger::instance();
//     LOG_DEBUG("Hello world!");
//     nlohmann::json obj;
//     obj["key"] = "value";
//     LOG_INFO("JSON: {}", obj.dump());
//     return 0;
// }

#include <QCoreApplication>

#include "log/logging.h"
#include "nlohmann/json.hpp"
#include "app/robotapp.h"

int main(int argc, char *argv[])
{
    Logger::instance();
    LOG_DEBUG("Hello world!");
    nlohmann::json obj;
    obj["key"] = "value";
    LOG_INFO("JSON: {}", obj.dump());

    QCoreApplication a(argc, argv);

    RobotApp app;
    app.init();
    app.start();

    // QCoreApplication *QCoreApplication::instance()

    return a.exec();
}

