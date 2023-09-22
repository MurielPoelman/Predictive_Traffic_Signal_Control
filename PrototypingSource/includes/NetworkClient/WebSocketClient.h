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

#include "exports.h"
#include <atomic>
#include "WebSocketEndpoint.h"

class NETWORKCLIENT_API WebSocketClient
{
public:
	WebSocketClient();
	~WebSocketClient();
	void SetHandler(IWebSocketClientHandlerPtr webSocketClientHandler);
	InterfaceWebSocketClientHandler* GetHandler();
	void SetUrl(const std::string& endpointUrl);
	bool Connect(std::chrono::seconds timeout = std::chrono::seconds(60));
	bool Send(const std::string& message);
	bool IsConnected();
	void Disconnect();
private:
	websocket_endpoint m_websocket_endpoint;
	std::atomic<int> m_connectionId;
	IWebSocketClientHandlerPtr m_webSocketClientHandler;
	std::string m_endpointUrl;
};
