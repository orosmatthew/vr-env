#include "network.hpp"

#include <asio.hpp>
#include <iostream>

struct LocalUdp::Impl {
    asio::io_context asio_context;
    asio::ip::udp::resolver asio_resolver;
    asio::ip::udp::resolver::query asio_query;
    asio::ip::udp::endpoint asio_endpoint;
    asio::ip::udp::socket asio_socket;

    Impl()
        : asio_resolver(asio_context)
        , asio_query(asio::ip::udp::v4(), "127.0.0.1", "7676")
        , asio_endpoint(*asio_resolver.resolve(asio_query))
        , asio_socket(asio_context)
    {
        asio_socket.open(asio::ip::udp::v4());
        asio_socket.non_blocking(true);
    }
};

void LocalUdp::Deleter::operator()(const Impl* p) const
{
    delete p;
}

LocalUdp::LocalUdp()
    : m_impl(new Impl)
{
}

void LocalUdp::send_data(const std::string& data) const
{
    try {
        m_impl->asio_socket.send_to(asio::buffer(data), m_impl->asio_endpoint);
    }
    catch (std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}