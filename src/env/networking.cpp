#include "networking.hpp"

#include <asio.hpp>
#include <iostream>

struct ReceiverUdp::Impl {
    asio::io_context asio_context;
    asio::ip::udp::resolver asio_resolver;
    asio::ip::udp::resolver::query asio_query;
    asio::ip::udp::endpoint asio_endpoint;
    asio::ip::udp::socket asio_socket;
    std::thread thread;
    std::array<char, 1024> buffer;
    size_t buffer_bytes;
    std::mutex buffer_mutes;
    std::atomic<bool> thread_stop;

    Impl()
        : asio_resolver(asio_context)
        , asio_query(asio::ip::udp::v4(), "127.0.0.1", "7676")
        , asio_endpoint(*asio_resolver.resolve(asio_query))
        , asio_socket(asio_context)
        , buffer()
        , buffer_bytes(0)
        , thread_stop(false)
    {
        asio_socket.open(asio::ip::udp::v4());
        asio_socket.bind(asio_endpoint);
        thread = std::thread(handle_receive, this);
    }
};

void ReceiverUdp::Deleter::operator()(Impl* p) const
{
    p->thread_stop = true;
    p->asio_socket.close();
    p->thread.join();
    delete p;
}
void ReceiverUdp::handle_receive(Impl* impl)
{
    while (!impl->thread_stop) {
        std::array<char, 1024> buffer; // NOLINT(*-pro-type-member-init)
        asio::error_code error;

        const size_t bytes_received
            = impl->asio_socket.receive_from(asio::buffer(buffer), impl->asio_endpoint, 0, error);

        if (!error) {
            std::lock_guard lock(impl->buffer_mutes);
            impl->buffer_bytes = bytes_received;
            impl->buffer = buffer;
        }
        else {
            if (!(impl->thread_stop && error.value() == 10004)) {
                std::cerr << "[ReceiveUdp] Error receiving data: " << error.message() << std::endl;
            }
        }
    }
}

ReceiverUdp::ReceiverUdp()
    : m_impl(new Impl)
{
}

std::pair<std::array<char, 1024>, size_t> ReceiverUdp::get_data(bool clear) const
{
    std::array<char, 1024> buffer; // NOLINT(*-pro-type-member-init)
    size_t bytes;
    {
        std::lock_guard lock(m_impl->buffer_mutes);
        buffer = m_impl->buffer;
        bytes = m_impl->buffer_bytes;
        if (clear) {
            m_impl->buffer_bytes = 0;
        }
    }
    return { buffer, bytes };
}
