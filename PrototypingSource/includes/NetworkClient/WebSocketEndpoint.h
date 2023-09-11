#pragma once

#include <future>
#include <mutex>
#include <unordered_map>
#include "InterfaceWebSocketClientHandler.h"

#define _WEBSOCKETPP_CPP11_THREAD_
#define _WEBSOCKETPP_CPP11_CHRONO_
#define _WEBSOCKETPP_CPP11_TYPE_TRAITS_
#define _WEBSOCKETPP_CPP11_RANDOM_DEVICE_
#define ASIO_STANDALONE

#pragma warning(push)  
#pragma warning(disable: 4267)  
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/common/thread.hpp>
#include <websocketpp/common/memory.hpp>
#pragma warning(pop)

enum class ConnectionState
{
	UNKNOWN,
	CONNECTING,
	OPEN,
	CLOSED,
	FAILED
};

struct ConnectResponse
{
	bool Connected;
	int ConnectionId;
	std::string ServerHeader;
	std::string ErrorMessage;
};

typedef websocketpp::client<websocketpp::config::asio_client> wsclient;
typedef std::promise<ConnectResponse> ConnectPromise;
typedef std::future<ConnectResponse> ConnectFuture;

class connection_metadata
{
public:
	typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

	connection_metadata(int id, websocketpp::connection_hdl hdl, const std::string& uri, HandleWSConnectFunction handleWSConnect, HandleWSMessageFunction handleWSMessage, HandleWSDisconnectFunction handleWSDisconnect, ConnectPromise connectPromise);
	~connection_metadata();
	void on_open(wsclient* wsclient, websocketpp::connection_hdl hdl);
	void on_fail(wsclient* wsclient, websocketpp::connection_hdl hdl);
	void on_close(wsclient* wsclient, websocketpp::connection_hdl hdl);
	void on_message(websocketpp::connection_hdl, wsclient::message_ptr msg);
	websocketpp::connection_hdl get_hdl() const;
	int get_id() const;
	ConnectionState get_status() const;
	std::string get_uri() const;
private:
	int m_id;
	websocketpp::connection_hdl m_hdl;
	ConnectionState m_status;
	std::string m_uri;
	ConnectPromise m_connectPromise;
	HandleWSMessageFunction m_handleWSMessage;
	HandleWSConnectFunction m_handleWSConnect;
	HandleWSDisconnectFunction m_handleWSDisconnect;
};

class websocket_endpoint
{
public:
	websocket_endpoint();
	~websocket_endpoint();
	ConnectFuture AsyncConnect(const std::string& uri, HandleWSConnectFunction handleWSConnect, HandleWSMessageFunction handleWSMessage, HandleWSDisconnectFunction handleWSDisconnect );
	void Close(int connectionId);
	bool Send(int connectionId, const std::string& message);
	bool IsConnected(int connectionId);
private:
	connection_metadata::ptr get_metadata(int connectionId) const;
	typedef std::unordered_map<int, connection_metadata::ptr> con_list;
	wsclient m_endpoint;
	websocketpp::lib::shared_ptr<websocketpp::lib::thread> m_thread;
	con_list m_connection_list;
	std::atomic<int> m_next_id;
};
