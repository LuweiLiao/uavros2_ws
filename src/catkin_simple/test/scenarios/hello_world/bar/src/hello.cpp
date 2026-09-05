#include <bar/hello.h>
#include <bar/msg/header_string.hpp>

#include <iostream>

#include <boost/thread.hpp>


inline void print_hello() {
    std::cout << "Hello ";
}

void hello() {
    boost::thread t(print_hello);
    t.join();
}
