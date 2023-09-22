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

#include "AimsunWSClientHandler.h"

AimsunWSClientHandler::AimsunWSClientHandler()
{
}

AimsunWSClientHandler::~AimsunWSClientHandler()
{
	RemoveAllReponseFilter();
}

void AimsunWSClientHandler::HandleMessage(const std::string& message)
{
	std::vector<std::string> filterterms;
	{
		std::lock_guard<std::mutex> lock(m_responseFilters_mutex);
		for (auto it = m_responseFilters.begin(); it != m_responseFilters.end(); )
		{
			filterterms.push_back(it->first);
			it++;
		}
	}

	for (auto filterterm : filterterms)
	{
		if (message.find(filterterm) != std::string::npos)
		{
			std::lock_guard<std::mutex> lock(m_responseFilters_mutex);
			m_responseFilters[filterterm].set_value(message);
			m_responseFilters.erase(filterterm);
			return;
		}
	}

	std::lock_guard<std::mutex> lock(m_messages_mutex);
	m_messages.push(message);
}

std::queue<std::string> AimsunWSClientHandler::getMessages()
{
	std::queue<std::string> newMessages;
	{
		std::lock_guard<std::mutex> lock(m_messages_mutex);
		newMessages.swap(m_messages);
	}

	return newMessages;
}

GetMessagesFunction AimsunWSClientHandler::GetFunctionGetMessages()
{
	return [this]() { return getMessages();	};
}

AddReponseFilterFunction AimsunWSClientHandler::GetFunctionAddReponseFilter()
{
	return [this](const std::string& reponseFilter, ResponsePromise responsePromise) { AddReponseFilter(reponseFilter, std::move(responsePromise)); };
}

CancelReponseFilterFunction AimsunWSClientHandler::GetFunctionCancelReponseFilter()
{
	return [this](const std::string& reponseFilter) { CancelReponseFilter(reponseFilter); };
}

void AimsunWSClientHandler::AddReponseFilter(const std::string& reponseFilter, ResponsePromise responsePromise)
{
	std::lock_guard<std::mutex> lock(m_responseFilters_mutex);
	if (m_responseFilters.find(reponseFilter) != m_responseFilters.end())
	{
		std::cout << "Try to add an already existing reponseFilter" << std::endl;
	}
	m_responseFilters[reponseFilter] = std::move(responsePromise);
}

void AimsunWSClientHandler::CancelReponseFilter(const std::string& reponseFilter)
{
	std::lock_guard<std::mutex> lock(m_responseFilters_mutex);
	if (m_responseFilters.find(reponseFilter) != m_responseFilters.end())
	{
		m_responseFilters[reponseFilter].set_value(std::string());
		m_responseFilters.erase(reponseFilter);
	} 
}

void AimsunWSClientHandler::RemoveAllReponseFilter()
{
	std::lock_guard<std::mutex> lock(m_responseFilters_mutex);
	for (auto it = m_responseFilters.begin(); it != m_responseFilters.end(); )
	{
		it->second.set_value(std::string());
		it++;
	}
	m_responseFilters.clear();
}

HandleWSMessageFunction AimsunWSClientHandler::GetFunctionHandleWSMessage()
{
	return [this](const std::string& message) { HandleMessage(message);	};
}

HandleWSConnectFunction AimsunWSClientHandler::GetFunctionHandleWSConnect()
{
	return [this](SendWSMessageFunction sendWSMessageFunction) { HandleConnect(sendWSMessageFunction); };
}

HandleWSDisconnectFunction AimsunWSClientHandler::GetFunctionHandleWSDisconnect()
{
	return [this]() { HandleDisconnect(); };
}

void AimsunWSClientHandler::HandleConnect(SendWSMessageFunction sendWSMessageFunction)
{
}

void AimsunWSClientHandler::HandleDisconnect()
{
}
