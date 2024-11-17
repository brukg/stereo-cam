#pragma once

#include <atomic>
#include <functional>
#include <list>
#include <mutex>
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>

struct event_base;

class EventLoop
{
public:
	EventLoop();
	~EventLoop();

	void exit(int code = 0);
	int exec(int width, int height, int timeout);

	void timeout(unsigned int sec);
	void callLater(const std::function<void()> &func);

private:
	static EventLoop *instance_;

	static void timeoutTriggered(int fd, short event, void *arg);

	struct event_base *event_;
	std::atomic<bool> exit_;
	int exitCode_;

	std::list<std::function<void()>> calls_;
	std::mutex lock_;

	void interrupt();
	void dispatchCalls();
};