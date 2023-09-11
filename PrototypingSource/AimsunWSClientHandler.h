#pragma once
#include "NetworkClient/InterfaceWebSocketClientHandler.h"
#include <queue>
#include <future>
#include <mutex>
#include <unordered_map>
#include <iostream>

typedef std::promise<std::string> ResponsePromise;
typedef std::future<std::string> ResponseFuture;

typedef std::function<void(const std::string& reponseFilter, ResponsePromise responsePromise)> AddReponseFilterFunction;
typedef std::function<void(const std::string& reponseFilter)> CancelReponseFilterFunction;
typedef std::function<std::queue<std::string>()> GetMessagesFunction;

class AimsunWSClientHandler : public InterfaceWebSocketClientHandler
{
public:
	AimsunWSClientHandler();
	virtual ~AimsunWSClientHandler();
	virtual HandleWSMessageFunction GetFunctionHandleWSMessage();
	virtual HandleWSConnectFunction GetFunctionHandleWSConnect();
	virtual HandleWSDisconnectFunction GetFunctionHandleWSDisconnect();
	GetMessagesFunction GetFunctionGetMessages();
	AddReponseFilterFunction GetFunctionAddReponseFilter();
	CancelReponseFilterFunction GetFunctionCancelReponseFilter();

private:
	void HandleMessage(const std::string& message);
	void HandleConnect(SendWSMessageFunction sendWSMessageFunction);
	void HandleDisconnect();
	void AddReponseFilter(const std::string& reponseFilter, ResponsePromise responsePromise);
	void CancelReponseFilter(const std::string& reponseFilter);
	void RemoveAllReponseFilter();
	std::queue<std::string> getMessages();

	std::mutex m_messages_mutex;
	std::queue<std::string> m_messages;

	std::mutex m_responseFilters_mutex;
	std::unordered_map<std::string /*responseFilter*/, ResponsePromise> m_responseFilters;
};
