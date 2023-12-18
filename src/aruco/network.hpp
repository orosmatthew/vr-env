#pragma once

#include <memory>
#include <string>

class LocalUdp {
public:
    LocalUdp();

    void send_data(const std::string& data) const;

private:
    struct Impl;
    struct Deleter {
        void operator()(const Impl* p) const;
    };

    std::unique_ptr<Impl, Deleter> m_impl;
};