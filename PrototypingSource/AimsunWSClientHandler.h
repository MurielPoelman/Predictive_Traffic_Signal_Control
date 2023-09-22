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
