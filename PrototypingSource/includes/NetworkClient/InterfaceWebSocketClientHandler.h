#pragma once
#include <functional>
#include <string>
#include <memory>
#include <thread>

typedef std::function<bool(const std::string& message)> SendWSMessageFunction;
typedef std::function<void(const std::string& message)> HandleWSMessageFunction;
typedef std::function<void(SendWSMessageFunction)> HandleWSConnectFunction;
typedef std::function<void()> HandleUpdateApplicationsFunction;
typedef std::function<void()> HandleWSDisconnectFunction;

class InterfaceWebSocketClientHandler 
{
public:
	virtual ~InterfaceWebSocketClientHandler() = default;

	virtual HandleWSMessageFunction GetFunctionHandleWSMessage() = 0;
	virtual HandleWSConnectFunction GetFunctionHandleWSConnect() = 0;
	virtual HandleWSDisconnectFunction GetFunctionHandleWSDisconnect() = 0;
};
typedef std::unique_ptr<InterfaceWebSocketClientHandler> IWebSocketClientHandlerPtr;