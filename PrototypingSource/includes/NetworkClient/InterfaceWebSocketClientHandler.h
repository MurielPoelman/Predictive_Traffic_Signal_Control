/*
Copyright(C) 2023 Royal HaskoningDHV / Path2Mobility B.V.

This program is free software : you can redistribute it and /or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.If not, see http ://www.gnu.org/licenses/.
*/

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