#pragma once

#include <memory>
#include <string>

class ReceiverUdp {
public:
    ReceiverUdp();

    [[nodiscard]] std::pair<std::array<char, 1024>, size_t> get_data(bool clear = true) const;

private:
    struct Impl;
    struct Deleter {
        void operator()(Impl* p) const;
    };

    static void handle_receive(Impl* impl);

    std::unique_ptr<Impl, Deleter> m_impl;
};