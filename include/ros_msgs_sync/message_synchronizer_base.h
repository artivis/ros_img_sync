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
  SubcriberParametersBase(const std::size_t queue_size) : queue_size_(queue_size) {}

  std::size_t queue_size_ = 10;
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

namespace detail{
template <typename T>
struct MakeSubscriber{
  template <typename... Args>
  static meta::add_shared_ptr_t<T> make_subscriber(ros::NodeHandle& nh, Args&&... args)
  {
    return meta::make_shared<T>(nh, std::forward<Args>(args)...);
  }
};

template <>
struct MakeSubscriber<typename Subscriber<sensor_msgs::Image>::type>{
  template <typename... Args>
  static SubscriberPtr<sensor_msgs::Image> make_subscriber(ros::NodeHandle& nh, Args&&... args)
  {
    image_transport::ImageTransport it(nh);
    image_transport::TransportHints th;
    return meta::make_shared<typename Subscriber<sensor_msgs::Image>::type>(it, std::forward<Args>(args)..., th);
  }
};
} /* namespace detail */

/**
* class MessageSynchronizerBase
* It synchronises ros messages topic callbacks (up to 8)
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

  using type = MessageSynchronizerBase<SyncPolicy, Args...>;

  using SynchronizerPolicy = SyncPolicy<Args...>;

  using Synchronizer = message_filters::Synchronizer<SynchronizerPolicy>;
  using SynchronizerPtr = meta::add_shared_ptr_t<Synchronizer>;

  using Subscribers = std::tuple<SubscriberPtr<Args>...>;

  using Messages = std::tuple<boost::shared_ptr<const Args>...>;

public:

  MessageSynchronizerBase();

  template <typename... SubParams>
  MessageSynchronizerBase(ros::NodeHandle& nh, SubParams&&...);

  template <typename... SubParams>
  MessageSynchronizerBase(SubParams&&...);

  virtual ~MessageSynchronizerBase() = default;

  bool start();

  inline const std::vector<std::string>& getTopics() const noexcept
  {
    return topics_;
  }

  inline const ros::NodeHandle& getNodeHandle() const noexcept
  {
    return nh_;
  }

  Messages getMessage() const
  {
    return messages_;
  }

protected:

  bool subs_instantiate_ = false;

  std::size_t sync_q_size_ = 10;
  ros::NodeHandle nh_ = ros::NodeHandle("~");

  std::vector<std::string> topics_;
  meta::RepTup<num_topics, SubcriberParameters> sub_params_;

  /// @brief Tuple of SubscriberPtr
  Subscribers subscribers_;

  /// @brief Topic synchronizer
  SynchronizerPtr synchronizer_;

  Messages messages_;

protected:

  virtual void callback(const boost::shared_ptr<const Args>&... /*args*/);

  // Loop over subs/topics to instantiate subscribers
  template<std::size_t I>
  void instantiateSubscribers(std::integral_constant<std::size_t, I>);
  void instantiateSubscribers(std::integral_constant<std::size_t, 0>);

  bool instantiateSubscribers();

  template <std::size_t... Indices>
  bool initSyncSubs(meta::index_sequence<Indices...>);

  template <std::size_t... Indices>
  void startAll(meta::index_sequence<Indices...>);

public:

  template <std::size_t I>
  void setQueueSize(const std::size_t queue_size)
  {
    std::get<I>(sub_params_).queue_size_ = queue_size;
  }

  template <std::size_t I>
  auto getSubscriberPtr()
  -> decltype(std::get<I>(std::declval<type>().subscribers_))
  {
    return std::get<I>(subscribers_);
  }

  template <std::size_t I>
  auto getSubscriberPtr() const
  -> decltype(getSubscriberPtr<I>())
  {
    return getSubscriberPtr<I>();
  }
};

template <template <typename...> class SyncPolicy, typename... Args>
template <typename... SubParams>
MessageSynchronizerBase<SyncPolicy, Args...>::MessageSynchronizerBase(SubParams&&... topics)
  : sub_params_(std::make_tuple(std::forward<SubParams>(topics)...))
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
}

template <template <typename...> class SyncPolicy,typename... Args>
template<std::size_t I>
void MessageSynchronizerBase<SyncPolicy, Args...>::instantiateSubscribers(std::integral_constant<size_t, I>)
{
  using SubTyp = meta::rm_shared_ptr_t<typename std::decay< decltype(std::get<I>(subscribers_) ) >::type>;

  std::get<I>(subscribers_) =
      detail::MakeSubscriber<SubTyp>::make_subscriber(
        nh_, "synchronized_topic_"+std::to_string(I), std::get<I>(sub_params_).queue_size_);

  instantiateSubscribers(std::integral_constant<size_t, I-1>());
}

template <template <typename...> class SyncPolicy,typename... Args>
void MessageSynchronizerBase<SyncPolicy, Args...>::instantiateSubscribers(std::integral_constant<size_t, 0>)
{
  using SubTyp = meta::rm_shared_ptr_t<typename std::decay< decltype(std::get<0>(subscribers_)) >::type>;

  std::get<0>(subscribers_) =
      detail::MakeSubscriber<SubTyp>::make_subscriber(
        nh_, "synchronized_topic_0", std::get<0>(sub_params_).queue_size_);
}
} /* namespace ros_msgs_sync */

#endif /* ROS_IMG_SYNC_MESSAGE_SYNCHRONIZER_BASE_H */
