#pragma once

#include <iostream>
#include <memory>

// ==============================================================================
// Logger class
// ==============================================================================

namespace fvlam
{

  class Logger
  {
  public:
    enum Levels
    {
      level_debug = 0,
      level_info,
      level_warning,
      level_error,
      level_fatal,
    };

    class CallbackInterface
    {
    public:
      virtual ~CallbackInterface() = default;

      // Return the current debugging level.
      virtual Levels level() = 0;

      // Return a stream at the specified debugging level.
      virtual std::basic_ostream<char> &stream(Levels level) = 0;
    };

    class StreamProxy
    {
      bool output_;
      std::basic_ostream<char> &ostream_;

    public:
      StreamProxy(CallbackInterface &callback, Levels level) :
        output_{level >= callback.level()},
        ostream_{callback.stream(level)}
      {}

      template<typename V>
      StreamProxy &operator<<(V const &value)
      {
        if (output_) {
          ostream_ << value;
        }
        return *this;
      }

      StreamProxy &operator<<(std::basic_ostream<char> &(*func)(std::basic_ostream<char> &))
      {
        if (output_) {
          func(ostream_);
        }
        return *this;
      }
    };

  private:
    std::shared_ptr<CallbackInterface> callback_;

  public:
    explicit Logger(std::shared_ptr<CallbackInterface> callback) :
      callback_{std::move(callback)}
    {}

    // These methods return a new StreamProxy object that will stream or not.
    // This temporary StreamProxy object is held alive while the subsequent << operators
    // pass a reference to it along. The StreamProxy object is then destroyed at the end
    // of the statement. This is convenient compiler magic.
    StreamProxy debug()
    { return StreamProxy{*callback_, level_debug}; } //
    StreamProxy info()
    { return StreamProxy{*callback_, level_info}; } //
    StreamProxy warning()
    { return StreamProxy{*callback_, level_warning}; } //
    StreamProxy error()
    { return StreamProxy{*callback_, level_error}; } //
    StreamProxy fatal()
    { return StreamProxy{*callback_, level_fatal}; } //


    auto output_debug() const
    { return level_debug >= callback_->level(); } //
    auto output_info() const
    { return level_info >= callback_->level(); } //
    auto output_warning() const
    { return level_warning >= callback_->level(); } //
    auto output_error() const
    { return level_error >= callback_->level(); } //
    auto output_fatal() const
    { return level_fatal >= callback_->level(); } //
  };

// ==============================================================================
// LoggerCallbackCout class
// ==============================================================================

  class LoggerCallbackCout : public Logger::CallbackInterface
  {
    Logger::Levels output_level_;

  public:
    explicit LoggerCallbackCout(Logger::Levels output_level) :
      output_level_{output_level}
    {}

    // Return the current debugging level.
    Logger::Levels level() override
    { return output_level_; }

    // Return a stream at the specified debugging level.
    std::basic_ostream<char> &stream(Logger::Levels level) override
    {
      (void) level;
      return std::cout;
    }
  };
}
