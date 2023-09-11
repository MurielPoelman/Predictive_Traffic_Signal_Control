#pragma once

#pragma warning (disable:4251)

#ifdef _WIN32
#    ifdef NETWORKCLIENT_EXPORTS
#        define NETWORKCLIENT_API __declspec(dllexport)
#    else
#        define NETWORKCLIENT_API __declspec(dllimport)
#    endif
#elif
#    define NETWORKCLIENT_API
#endif

