//
// Created by yongxi on 8/31/21.
//

#include "bullet_client/bullet_client.h"
#include "log4cxx/logger.h"

log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger("BulletClient");

BulletClient::BulletClient() {
    client_ = new b3RobotSimulatorClientAPI();
    if(client_->connect(eCONNECT_GUI)) {
        LOG4CXX_ERROR(logger, "connect to bullet3 client failed");
    }
}