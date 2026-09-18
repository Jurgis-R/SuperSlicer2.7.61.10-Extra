///|/ Copyright (c) SuperSlicer 2026 Durand Rémi @supermerill
///|/
///|/ SuperSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_
#define slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_

/*
Deferred exceptions at C callback boundaries
============================================

A C or ctypes callback must never let a C++ exception unwind through the
foreign caller. DeferredCallbackException lets such a callback catch the
exception, remember the first failure, return normally through the C ABI, and
rethrow only after execution has returned to a C++ frame.

The object deliberately does not interpret the exception. Its owner decides
whether the captured value means cancellation, an invalid argument, or an
ordinary failure, and may perform immediate side effects such as requesting
cooperative cancellation. Keeping that policy outside this helper allows the
same boundary mechanism to serve callbacks with different result protocols.

Typical use:

    DeferredCallbackException deferred;
    c_api_call([](void *context) noexcept {
        DeferredCallbackException &failure =
            *static_cast<DeferredCallbackException *>(context);
        if (failure.captured())
            return;
        try {
            callback_work();
        } catch (...) {
            failure.capture_current();
        }
    }, &deferred);
    deferred.rethrow_if_captured();

capture_current() must be called from a catch handler. The first exception is
retained so a later cleanup failure cannot hide the operation's primary cause.
*/

#include <exception>
#include <utility>

namespace slic3r_api {

class DeferredCallbackException
{
public:
    // Capture the exception currently handled by the caller. Return true only
    // when this call stored the first exception; false means that an earlier
    // exception already owns the outcome or that no exception is being handled.
    bool capture_current() noexcept
    {
        if (m_exception != nullptr)
            return false;

        std::exception_ptr current = std::current_exception();
        if (current == nullptr)
            return false;

        m_exception = std::move(current);
        return true;
    }

    bool captured() const noexcept { return m_exception != nullptr; }

    // Call this only after the foreign invocation has returned. Rethrowing
    // while still inside the C callback would violate the boundary this class
    // is intended to protect.
    void rethrow_if_captured() const
    {
        if (m_exception != nullptr)
            std::rethrow_exception(m_exception);
    }

private:
    std::exception_ptr m_exception;
};

} // namespace slic3r_api

#endif // slic3r_Api_plugin_cpp_DeferredCallbackException_hpp_
