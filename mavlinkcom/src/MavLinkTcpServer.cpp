// Copyright (C) Microsoft Corporation. 
// Copyright (C) IAMAI Consulting Corporation.  

// MIT License. All rights reserved.

#include "MavLinkTcpServer.hpp"

#include "impl/MavLinkTcpServerImpl.hpp"

using namespace mavlinkcom;
using namespace mavlinkcom_impl;

MavLinkTcpServer::MavLinkTcpServer(const std::string& local_addr,
                                   int local_port)
    : impl_(new MavLinkTcpServerImpl(local_addr, local_port)) {}

std::shared_ptr<MavLinkConnection> MavLinkTcpServer::acceptTcp(
    const std::string& nodeName) {
  return impl_->acceptTcp(nodeName);
}

MavLinkTcpServer::MavLinkTcpServer() {}
MavLinkTcpServer::~MavLinkTcpServer() {}
// MavLinkTcpServer::MavLinkTcpServer(MavLinkTcpServer&&) = default;
// MavLinkTcpServer& MavLinkTcpServer::operator=(MavLinkTcpServer&&) = default;
