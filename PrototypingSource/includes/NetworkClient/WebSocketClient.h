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
