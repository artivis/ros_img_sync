/** \author Jeremie Deray. */

#ifndef ROS_IMG_SYNC_MESSAGE_SYNCHRONIZER_BASE_H
#define ROS_IMG_SYNC_MESSAGE_SYNCHRONIZER_BASE_H

#include "ros_msgs_sync/meta_utils.h"

// ROS headers
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/Image.h>
#include <image_transport/subscriber_filter.h>

namespace ros_msgs_sync{

namespace detail{
static const auto PlaceHolders = std::make_tuple(_1, _2, _3, _4, _5, _6, _7, _8, _9);

struct StampPrinter
{
  StampPrinter(std::stringstream& ss) : ss_(ss) {}

  template<typename T>
  void operator () (T&& t) const
  {
    ss_ << t->header.stamp << " ";
  }

private:
  std::stringstream& ss_;
};
} /* namespace detail */

/**
 * @brief The SubcriberParameters struct
 */
struct SubcriberParametersBase
{
  SubcriberParametersBase(const std::size_t _queue_size) : queue_size(_queue_size) {}

  std::size_t queue_size = 10;
};

/**
 * @brief The SubcriberParameters struct
 */
struct SubcriberParameters : SubcriberParametersBase
{
  SubcriberParameters(const std::size_t queue_size) : SubcriberParametersBase(queue_size) {}
  SubcriberParameters(const std::size_t queue_size, const ros::TransportHints& th)
    : SubcriberParametersBase(queue_size), transport_hints(th) {}

  ros::TransportHints transport_hints = ros::TransportHints();
  ros::CallbackQueueInterface* callback_queue = nullptr;
};

/**
 * @brief The SubcriberParameters struct
 */
struct ImageSubcriberParameters : SubcriberParametersBase
{
  ImageSubcriberParameters(const std::size_t queue_size) : SubcriberParametersBase(queue_size) {}

  image_transport::TransportHints transport_hints = image_transport::TransportHints();
};

template <typename T>
struct SubscriberParametersTraits
{
  using type = SubcriberParameters;
};

template <>
struct SubscriberParametersTraits<sensor_msgs::Image>
{
  using type = ImageSubcriberParameters;
};

template <typename T> struct Subscriber {
  using type  = message_filters::Subscriber<T>;
  using msg_t = T;
  using Ptr = meta::add_shared_ptr_t<type>;
};

template <> struct Subscriber<sensor_msgs::Image> {
  using type = image_transport::SubscriberFilter;
  using msg_t = sensor_msgs::Image;
  using Ptr = meta::add_shared_ptr_t<type>;
};

template <typename T> using SubscriberPtr = typename Subscriber<T>::Ptr;

namespace detail {

template <typename T>
struct MakeSubscriber{
  static meta::add_shared_ptr_t<T> make_subscriber(ros::NodeHandle& nh,
                                                   const std::string& topic,
                                                   const SubcriberParameters& params)
  {
    return meta::make_shared<T>(nh, topic, params.queue_size,
                                params.transport_hints, params.callback_queue);
  }
};

template <>
struct MakeSubscriber<typename Subscriber<sensor_msgs::Image>::type>{
  static SubscriberPtr<sensor_msgs::Image> make_subscriber(ros::NodeHandle& nh,
                                                           const std::string& topic,
                                                           const ImageSubcriberParameters& params)
  {
    image_transport::ImageTransport it(nh);
    return meta::make_shared<typename Subscriber<sensor_msgs::Image>::type>(
          it, topic, params.queue_size, params.transport_hints);
  }
};

} /* namespace detail */

/**
* @brief MessageSynchronizerBase
* It synchronises ros messages topic callbacks (up to 9)
* Its callback is pure virtual so that it can be easily
* defined in a derived class
*/
template <template <typename...> class SyncPolicy, typename... Args>
class MessageSynchronizerBase
{
public:

  static constexpr std::size_t num_topics = sizeof...(Args);

  static constexpr std::size_t SYNCHRONIZER_LIMIT = 9;

  static_assert(std::integral_constant<bool, (num_topics <= SYNCHRONIZER_LIMIT)>::value,
                "Too many template arguments, max is 9 !");

private:

  using NullType = message_filters::NullType;

  using PolicyBaseArgs = meta::tuple_cat_t<std::tuple<Args...>, meta::RepTup<9-num_topics, NullType>>;

  template <typename T> struct make_policy_base;

  template <typename... Ts>
  struct make_policy_base<std::tuple<Ts...>>
  {
    static_assert(sizeof...(Ts)==9, "Class message_filters::PolicyBase expects 9 template arguments !");
    using type = message_filters::PolicyBase<Ts...>;
  };

  using PolicyBase = typename make_policy_base<PolicyBaseArgs>::type;

  static_assert(std::is_base_of<PolicyBase, SyncPolicy<Args...>>::value,
                "Template parameter SyncPolicy must inherit from message_filters::sync_policies::PolicyBase !");

public:

  /// @brief the type of this class
  using type = MessageSynchronizerBase<SyncPolicy, Args...>;

  using SynchronizerPolicy = SyncPolicy<Args...>;

  using Synchronizer = message_filters::Synchronizer<SynchronizerPolicy>;
  using SynchronizerPtr = meta::add_shared_ptr_t<Synchronizer>;

  using Subscribers = std::tuple<SubscriberPtr<Args>...>;

  using SubscribersParameters = std::tuple<typename SubscriberParametersTraits<Args>::type...>;

  using Messages = std::tuple<boost::shared_ptr<const Args>...>;

public:

  /// @note Explicit: No default constructor
  MessageSynchronizerBase() = delete;
  virtual ~MessageSynchronizerBase() = default;

  /**
   * @brief Constructor given a ros::NodeHandle and the
   * parameters of the subscribers
   * @param nh, the internal ros::NodeHandle
   * @param params, the subscribers parameters
   * @see SubcriberParameters
   * @see ImageSubcriberParameters
   */
  template <typename... SubParams>
  MessageSynchronizerBase(ros::NodeHandle& nh, SubParams&&... params);

  /**
   * @brief Constructor given the
   * parameters of the subscribers
   * @param params, the subscribers parameters
   * @see SubcriberParameters
   * @see ImageSubcriberParameters
   */
  template <typename... SubParams>
  MessageSynchronizerBase(SubParams&&...);

  /// @brief start the synchronization
  /// @return true if started
  bool start();

  /// @brief stop the synchronization
  /// @return true if stopped
  bool stop();

  /// @brief re-start the synchronization
  /// @return true if started
  bool restart();

  /// @brief Whether the synchronization is running or not
  /// @return true if running
  bool running() const noexcept;

  /// @brief Get the Synchronizer ros::NodeHandle
  /// @return Synchronizer's ros::NodeHandle
  const ros::NodeHandle& getNodeHandle() const noexcept;

  /// @brief Get the synchronized messages
  /// @return Messages a std::tuple of
  /// const boost::shared_ptr<MsgType>
  Messages getMessage() const;

  /// @brief Wait until the first synchronized messages
  /// are received.
  /// @param sleep, Rate a which the reception is evaluated
  void wait(ros::Rate sleep = ros::Rate(30)) const;

  /// @brief Wait during d secs for the first synchronized messages
  /// @param d, The max duration to waiting for the first messages
  /// @param sleep, Rate a which the reception is evaluated
  bool wait(const ros::Duration d,
            ros::Rate sleep = ros::Rate(30)) const;

  /// @brief Get the synchronizer callback queue size
  /// @return the synchronizer callback queue size
  std::size_t getSyncQueueSize() const noexcept;

  /// @brief Set the synchronizer callback queue size
  /// @param size, the synchronizer callback queue size
  void setSyncQueueSize(const std::size_t size);

protected:

  bool subs_instantiate_ = false;
  bool first_received_ = false;

  std::size_t sync_q_size_ = 10;
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  /// @brief std::tuple of SubscriberPtr
  Subscribers subscribers_;

  /// @brief std::tuple of SubcriberParameters/ImageSubcriberParameters
  SubscribersParameters parameters_;

  /// @brief Topic synchronizer
  /// @see message_filters::Synchronizer
  SynchronizerPtr synchronizer_;

  /// @brief std::tuple of
  /// const boost::shared_ptr<MsgType>
  Messages messages_;

protected:

  /// @brief Synchronized messages callback
  /// @note virtual function
  virtual void callback(const boost::shared_ptr<const Args>&... /*args*/);

  /// @brief Helper function to loop over subscriber
  /// and instantiate them
  template<std::size_t I>
  void instantiateSubscribers(std::integral_constant<std::size_t, I>);
  /// @brief Helper function to loop over subscriber
  /// and instantiate them
  void instantiateSubscribers(std::integral_constant<std::size_t, 0>);

  /// @brief Instantiate subscribers
  bool instantiateSubscribers();

  /// @brief Helper function to instantiate the
  /// message_filters::Synchronizer
  template <std::size_t... Indices>
  bool initSyncSubs(meta::index_sequence<Indices...>);

public:

  /// @brief Set the callback queue size of the Ith
  /// subscriber
  /// @param I, subscriber index
  /// @param _queue_size, callback queue size
  template <std::size_t I>
  void setQueueSize(const std::size_t _queue_size);

  /// @brief Get the Ith subscriber
  /// @param I, subscriber index
  template <std::size_t I>
  auto getSubscriberPtr() const
  -> const typename std::tuple_element<I, Subscribers>::type&;

  /// @brief Get the Ith subscriber's parameters
  /// @param I, subscriber index
  template <std::size_t I>
  auto getSubscriberParameters() const
  -> const typename std::tuple_element<I, SubscribersParameters>::type&;

  /// @brief Get the Ith subscriber's parameters
  /// @param I, subscriber index
  template <std::size_t I>
  auto getSubscriberParameters()
  -> typename std::tuple_element<I, SubscribersParameters>::type&;
};

template <template <typename...> class SyncPolicy, typename... Args>
template <typename... SubParams>
MessageSynchronizerBase<SyncPolicy, Args...>::MessageSynchronizerBase(SubParams&&... params)
  : parameters_(std::make_tuple(std::forward<SubParams>(params)...))
{
  static_assert(sizeof...(SubParams) == num_topics,
                "The number of provided topics mismatches the number of template parameters.");
}

template <template <typename...> class SyncPolicy, typename... Args>
template <typename... SubParams>
MessageSynchronizerBase<SyncPolicy, Args...>::MessageSynchronizerBase(ros::NodeHandle& nh, SubParams&&... topics)
  : MessageSynchronizerBase(std::forward<SubParams>(topics)...)
{
  nh_ = nh;
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::start()
{
  bool ok = true;

  if (!subs_instantiate_)
  {
    ok = instantiateSubscribers();
    ok += initSyncSubs(meta::make_index_sequence<num_topics>());
  }

  return ok;
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::stop()
{
  if (subs_instantiate_)
  {
    synchronizer_.reset();
    subscribers_ = Subscribers{};
    subs_instantiate_ = false;
    first_received_ = false;
  }

  return !subs_instantiate_;
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::restart()
{
  stop();
  return start();
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::running() const noexcept
{
  return subs_instantiate_;
}

template <template <typename...> class SyncPolicy,typename... Args>
inline const ros::NodeHandle&
MessageSynchronizerBase<SyncPolicy, Args...>::getNodeHandle() const noexcept
{
  return nh_;
}

template <template <typename...> class SyncPolicy,typename... Args>
typename MessageSynchronizerBase<SyncPolicy, Args...>::Messages
MessageSynchronizerBase<SyncPolicy, Args...>::getMessage() const
{
  return messages_;
}

template <template <typename...> class SyncPolicy,typename... Args>
void MessageSynchronizerBase<SyncPolicy, Args...>::wait(ros::Rate sleep) const
{
  while (ros::ok() && !first_received_)
  {
    sleep.sleep();
  }
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::wait(const ros::Duration d,
                                                        ros::Rate sleep) const
{
  const auto start = ros::Time::now();
  while (ros::ok() && !first_received_ ||
         (ros::Time::now() - start) < d)
  {
    sleep.sleep();
  }
  return first_received_;
}

template <template <typename...> class SyncPolicy,typename... Args>
std::size_t MessageSynchronizerBase<SyncPolicy, Args...>::getSyncQueueSize() const noexcept
{
  return sync_q_size_;
}

template <template <typename...> class SyncPolicy,typename... Args>
void MessageSynchronizerBase<SyncPolicy, Args...>::setSyncQueueSize(const std::size_t size)
{
  sync_q_size_ = size;
}

template <template <typename...> class SyncPolicy,typename... Args>
bool MessageSynchronizerBase<SyncPolicy, Args...>::instantiateSubscribers()
{
  instantiateSubscribers(std::integral_constant<size_t, num_topics-1>());

  subs_instantiate_ = true;

  return subs_instantiate_;
}

template <template <typename... Args> class SyncPolicy,typename... Args>
template <std::size_t... Indices>
bool MessageSynchronizerBase<SyncPolicy, Args...>::initSyncSubs(meta::index_sequence<Indices...>)
{
  synchronizer_ = meta::make_shared<Synchronizer>( SynchronizerPolicy(sync_q_size_), *std::get<Indices>(subscribers_)... );

  synchronizer_->registerCallback(boost::bind(&MessageSynchronizerBase<SyncPolicy, Args...>::callback, this,
                                              std::get<Indices>( detail::PlaceHolders )...));
  return true;
}

template <template <typename...> class SyncPolicy,typename... Args>
void MessageSynchronizerBase<SyncPolicy, Args...>::callback(const boost::shared_ptr<const Args>&... args)
{
  messages_ = std::make_tuple(args...);

  std::stringstream ss;
  meta::for_each(std::forward_as_tuple<const boost::shared_ptr<const Args>&...>(args...),
                 detail::StampPrinter(ss));

  ROS_INFO_STREAM("[Default callback] Received " << sizeof...(Args) << " synchronized messages with stamps :");
  ROS_INFO_STREAM(ss.str() << "\n");

  first_received_ = true;
}

template <template <typename...> class SyncPolicy,typename... Args>
template<std::size_t I>
void MessageSynchronizerBase<SyncPolicy, Args...>::instantiateSubscribers(std::integral_constant<size_t, I>)
{
  using SubTyp = meta::rm_shared_ptr_t<typename std::decay< decltype(std::get<I>(subscribers_) ) >::type>;

  std::get<I>(subscribers_) =
      detail::MakeSubscriber<SubTyp>::make_subscriber(
        nh_, "synchronized_topic_"+std::to_string(I), std::get<I>(parameters_));

  instantiateSubscribers(std::integral_constant<size_t, I-1>());
}

template <template <typename...> class SyncPolicy,typename... Args>
void MessageSynchronizerBase<SyncPolicy, Args...>::instantiateSubscribers(std::integral_constant<size_t, 0>)
{
  using SubTyp = meta::rm_shared_ptr_t<typename std::decay< decltype(std::get<0>(subscribers_)) >::type>;

  std::get<0>(subscribers_) =
      detail::MakeSubscriber<SubTyp>::make_subscriber(
        nh_, "synchronized_topic_0", std::get<0>(parameters_));
}

template <template <typename...> class SyncPolicy,typename... Args>
template <std::size_t I>
void MessageSynchronizerBase<SyncPolicy, Args...>::setQueueSize(const std::size_t _queue_size)
{
  std::get<I>(parameters_).queue_size = _queue_size;
}

template <template <typename...> class SyncPolicy,typename... Args>
template <std::size_t I>
auto MessageSynchronizerBase<SyncPolicy, Args...>::getSubscriberPtr() const
-> const typename std::tuple_element<I, Subscribers>::type&
{
  static_assert(I <= num_topics, "Index I exceed number of subscribers !");
  return std::get<I>(subscribers_);
}

template <template <typename...> class SyncPolicy,typename... Args>
template <std::size_t I>
auto MessageSynchronizerBase<SyncPolicy, Args...>::getSubscriberParameters() const
-> const typename std::tuple_element<I, SubscribersParameters>::type&
{
  static_assert(I <= num_topics, "Index I exceed number of subscribers !");
  return std::get<I>(parameters_);
}

template <template <typename...> class SyncPolicy,typename... Args>
template <std::size_t I>
auto MessageSynchronizerBase<SyncPolicy, Args...>::getSubscriberParameters()
-> typename std::tuple_element<I, SubscribersParameters>::type&
{
  static_assert(I <= num_topics, "Index I exceed number of subscribers !");
  return std::get<I>(parameters_);
}

} /* namespace ros_msgs_sync */

#endif /* ROS_IMG_SYNC_MESSAGE_SYNCHRONIZER_BASE_H */
