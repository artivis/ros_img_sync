/*
* Software License Agreement (Modified BSD License)
*
* Copyright (c) 2014, PAL Robotics, S.L.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of PAL Robotics, S.L. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
/** \author Jeremie Deray. */

#ifndef ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H
#define ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H

// ROS headers
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Boost headers
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/variant.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>

// std header
#include <vector>
#include <tuple>




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <utility>
#include <type_traits>
#include <memory>
#include <tuple>

//////////////////////////
// cpp11 index_sequence //
//////////////////////////

template <std::size_t... Ints>
struct index_sequence
{
    using type = index_sequence;
    using value_type = std::size_t;
    static constexpr std::size_t size() noexcept { return sizeof...(Ints); }
};

// --------------------------------------------------------------

template <class Sequence1, class Sequence2>
struct _merge_and_renumber;

template <std::size_t... I1, std::size_t... I2>
struct _merge_and_renumber<index_sequence<I1...>, index_sequence<I2...>>
  : index_sequence<I1..., (sizeof...(I1)+I2)...>
{ };

 // --------------------------------------------------------------

template <std::size_t N>
struct make_index_sequence
  : _merge_and_renumber<typename make_index_sequence<N/2>::type,
                        typename make_index_sequence<N - N/2>::type>
{ };

template<> struct make_index_sequence<0> : index_sequence<>  { };
template<> struct make_index_sequence<1> : index_sequence<0> { };

template <typename T>
using shared_pointer = boost::shared_ptr<T>;

template <typename T, typename... Args>
shared_pointer<T> make_shared(Args&&... args)
{
  return boost::make_shared<T>(std::forward<Args>(args)...);
}

//////////////////////////
// Helper to add shared_ptr to type //
//////////////////////////

template <typename T>
struct add_shared_ptr
{
  using type = shared_pointer<T>;
};

template <typename T>
using add_shared_ptr_t = typename add_shared_ptr<T>::type;

//////////////////////////
// Helper to remove shared_ptr from type //
//////////////////////////

template <typename T>
struct rm_shared_ptr;

template <typename T>
struct rm_shared_ptr< shared_pointer<T> >
{
  using type = T;
};

template <typename T>
using rm_shared_ptr_t = typename rm_shared_ptr<T>::type;

//////////////////////////
// Helper to add shared_ptr to each type in tuple //
//////////////////////////

template <typename T>
struct to_each_add_shared_ptr;

template <typename... Args>
struct to_each_add_shared_ptr<std::tuple<Args...>>
{
  using type = std::tuple<add_shared_ptr_t<Args>...>;
};

//template <typename F, typename T>
//struct apply;

//template <template<class...> class F, typename... Args>
//struct apply<F<>, std::tuple<Args...>>
//{
//  using type = std::tuple<typename F<Args>::type...>;
//};

//template <typename... T>
//struct type_list { static constexpr std::size_t size = sizeof...(T); };

//template<typename List, typename T>
//struct append;

//template<typename... TL, typename T>
//struct append <type_list<TL...>, T> {
//    using type = type_list<TL..., T>;
//};

//template<typename L, typename R>
//using append_t = typename append<L,R>::type;

//template <typename... Args>
//struct Synchro
//{

//};

//template <typename... Args>
//using SynchroPtr = add_shared_ptr_t<Synchro<Args...>>;

//////////////////////////
// Helper to create a tuple with n args in variadic pack //
//////////////////////////

template <typename T, typename S>
struct subTuple;

template <template <std::size_t...> class S, std::size_t... Indices, typename... Args>
struct subTuple < std::tuple<Args...>, S<Indices...> >
{
    using type = decltype( std::make_tuple (std::get<Indices>( std::declval<std::tuple<Args...>>() )...) );
};

//template <typename T>
//struct tupleToSynchro;

//template <typename... T>
//struct tupleToSynchro<std::tuple<T...>>
//{
//    using type = Synchro<T...>;
//};

//template <std::size_t I, typename... Args>
//struct toSynchro
//{
//    using tuple_t = std::tuple<Args...>;

//    using i_seq = typename make_index_sequence<I>::type;

//    using type = typename tupleToSynchro<typename subTuple<tuple_t, i_seq>::type>::type;
//};

//////////////////////////
// Helper to concat tuple //
//////////////////////////

template<typename L, typename R>
struct tuple_cat;

template<typename... TL, typename... TR>
struct tuple_cat<std::tuple<TL...>, std::tuple<TR...>> {
    using type = std::tuple<TL..., TR...>;
};

//template <std::size_t I, typename... Args>
//struct toSynchros_
//{
//    using type = tuple_cat< std::tuple< typename toSynchro<I, Args...>::type >,
//                            typename toSynchros_<I-1, Args...>::type>;
//};

//template <typename... Args>
//struct toSynchros_<1, Args...>
//{
//  using type = std::tuple<>;
//};

//template <typename... Args>
//struct toSynchros : toSynchros_<sizeof...(Args), Args... >
//{

//};

//using test_t = toSynchro<2, int, int, char>::type;

//using test_t2 = typename toSynchros<int, double, char>::type;

//using tt= test_t2::TT;

//static_assert(std::is_same<test_t, Synchro<int, int> >::value, "");

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{
  namespace sp = message_filters::sync_policies;

  template <typename T>
  struct filtered_vector :
    public std::vector<boost::shared_ptr<T const> >
  {
      filtered_vector(const boost::shared_ptr<T const>& t)
      {
        if (t.get() != NULL)
          (*this)(t);
      }
      filtered_vector& operator()(const boost::shared_ptr<T const>& t)
      {
          if (t.get() != NULL)
            this->push_back(t);

          return *this;
      }
  };
}



/**
* class SyncImplHandler
* It synchronises ros messages topic callbacks (up to 8)
* Its callback is pure virtual so that it can be easily
* defined in a derived class
*/
template <typename... Args>
class SyncImplHandler
{
public:

  constexpr std::size_t SYNCHRONIZER_LIMIT = 8;

  static_assert(std::integral_constant<bool, (sizeof...(Args) <= SYNCHRONIZER_LIMIT)>::value,
                "Too many template arguments !");

  template <typename... Args>
  using SynchronizerPolicy = message_filters::sync_policies::ApproximateTime<Args...>;

  template <typename... Args>
  using SynchronizerPolicyPtr = add_shared_ptr_t<SynchronizerPolicy<Args...>>;

  template<class T> struct traits_synchronizer
  {
    using type = message_filters::Synchronizer<T>;
  };

  /**
  * Constructor.
  * Retrieve rosparam 'topics' as a list of image topics to synchronise
  *                   'queue_size' size of synronisation queue
  */
  SyncImplHandler();

  virtual ~SyncImplHandler() = default;

  bool start();

  inline const std::vector<std::string>& getTopics() const noexcept
  {
    return topics_;
  }

protected:

  template<class T>
  using Synchronizer = message_filters::Synchronizer<T>;

  typedef boost::function<void (const std::tuple<add_shared_ptr<Args>...>&)> callbackPtr;

  template <typename T>
  struct tupleToSyncPolicy;

  template <typename... T>
  struct tupleToSyncPolicy<std::tuple<T...>>
  {
      using type = SynchronizerPolicy<T...>;
  };

  template <std::size_t I, typename... Args>
  struct toSyncPolicy
  {
      using tuple_t = std::tuple<Args...>;

      using i_seq = typename make_index_sequence<I>::type;

      using type = typename tupleToSyncPolicy<typename subTuple<tuple_t, i_seq>::type>::type;
  };

  template <std::size_t I, typename... Args>
  struct toSyncPolicies_
  {
      using type = tuple_cat< std::tuple< typename toSyncPolicy<I, Args...>::type >,
                              typename toSyncPolicies_<I-1, Args...>::type>;
  };

  template <typename... Args>
  struct toSyncPolicies_<1, Args...>
  {
    using type = std::tuple<>;
  };

  template <typename... Args>
  struct toSyncPolicies : toSyncPolicies_<sizeof...(Args), Args... > { };

  template <typename T>
  struct toSynchronizerVariant;

  template <typename... T>
  struct toSynchronizerVariant<std::tuple<T...>>
  {
      using type = boost::variant< add_shared_ptr_t<message_filters::Synchronizer<T>>...>;
  };

  using SyncPolicies = typename toSyncPolicies<Args...>::type;

  using SynchronizerVariant = typename toSynchronizerVariant< SyncPolicies >::type;

  template<std::size_t I, typename... Ts, typename... Args>
  void instantiate_subscribers(std::integral_constant<size_t, I>, std::tuple<Ts...>& t, const std::vector<std::string>& topics)
  {
    std::get<I>(t) =
        make_shared< rm_shared_ptr_t<typename std::decay< decltype(std::get<I>(t) )>::type> >( topics[I] ) ;

    instantiate_subscribers(std::integral_constant<size_t, I-1>(), t, topics);
  }

  template<typename... Ts, typename... Args>
  void instantiate_subscribers(std::integral_constant<size_t, 0>, std::tuple<Ts...>& t, const std::vector<std::string>& topics)
  {
    std::get<0>(t) =
        make_shared< rm_shared_ptr_t<typename std::decay< decltype(std::get<0>(t) )>::type> >( topics[0] ) ;
  }

  template<typename... Ts, typename... Args>
  void instantiate_subscribers(std::tuple<Ts...>& t, const std::vector<std::string>& topics)
  {
    if (topics.size() != sizeof...(Args))
    {
      throw std::runtime_error("Provided " + std::to_string(topics.size()) + " topics"
                               " for " + std::to_string(sizeof...(Args)) + " subscribers !");
    }

    instantiate_subscribers(std::integral_constant<size_t, sizeof...(Ts)-1>(), t, topics);
  }

  /**
  * A pure virtual member.
  * @param vecMPtr : std::vector< M >
  *        callback has to be defined in derived class !
  */
  virtual void callback(const std::vector<MPtr>& vecMPtr) = 0;

  void wrapCallback(const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&,
                    const MPtr&, const MPtr&);

  virtual void initParam();
  virtual bool initSubs();
  virtual bool initSyncSubs();

  callbackPtr callback_ptr_;

  boost::shared_ptr<SynchronizerVariant> synchronizer_;

  /// @todo one q per topic
  std::size_t q_size_ = 10;
  ros::NodeHandle private_nh_ = ros::NodeHandle("~");

  std::vector<std::string> topics_;

  std::tuple<add_shared_ptr_t<message_filters::Subscriber<Args>>...> subscribers_;
};

template <typename... Args>
SyncImplHandler<Args...>::SyncImplHandler() :
  private_nh_("~"),
  q_size_(10)
{
  //
}

template <typename... Args>
bool SyncImplHandler<Args...>::start()
{
  initParam();

  bool ok = initSubs();

  ok += initSyncSubs();

  callback_ptr_ = boost::bind(&SyncImplHandler<Args...>::callback, this, _1);

  return ok;
}

template <typename... Args>
void SyncImplHandler<Args...>::initParam()
{
  if (!private_nh_.getParam("topics", topics_))
  {
    ROS_ERROR("No topics parameter specified !");
    return;
  }

  if (topics_.size() < 2)
  {
    ROS_ERROR("Only one topic specified !");
    return;
  }
  else if (topics_.size() > 8)
  {
    ROS_ERROR("You can only synchronize up to 8 topics, %i requested !", topics_.size());
    return;
  }

  private_nh_.param("queue_size", q_size_, q_size_);

  ROS_INFO_STREAM("Synchronizer queue size : " << q_size_);
  ROS_INFO("About to synchronize topics :\n");
  for (const auto& t : topics_)
    ROS_INFO("%s\n", t.c_str());
}

template <typename... Args>
bool SyncImplHandler<Args...>::initSubs()
{
  std::vector<std::string> topics = topics_;
  topics.resize(SYNCHRONIZER_LIMIT, "");

  instantiate_subscribers(subscribers_, topics);

  return true;
}

template <typename... Args>
void SyncImplHandler<Args...>::wrapCallback(const MPtr& a, const MPtr& b,
                                            const MPtr& c, const MPtr& d,
                                            const MPtr& e, const MPtr& f,
                                            const MPtr& g, const MPtr& h)
{
  std::vector<MPtr> vecMPtr = filtered_vector<M>(a)(b)(c)(d)(e)(f)(g)(h);

  callback_ptr_(vecMPtr);
}

template <typename... Args>
bool SyncImplHandler<Args...>::initSyncSubs()
{

  switch (topics_.size())
  {
    case 2:
      using Policy = typename std::decay<decltype( std::get<1>(std::declval<SyncPolicies>()) )>::type;
      using Sync = Synchronizer<Policy>;
      synchronizer_ = make_shared<SynchronizerVariant>( make_shared<Sync>(Policy(q_size_), topics_[0], topics_[1]) );
      //
      break;
    case 3:
      using Policy = typename std::decay<decltype( std::get<2>(std::declval<SyncPolicies>()) )>::type;
      using Sync = Synchronizer<Policy>;
      synchronizer_ = make_shared<SynchronizerVariant>( make_shared<Sync>(Policy(q_size_), topics_[0], topics_[1], topics_[2]) );
      //
      break;
  }

//  switch (topics_.size())
//  {
//    case 2:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync2>::Ptr(new typename Sync<ApproxSync2>::type(ApproxSync2(q_size_),
//                                                                                                                 _msg_subs[0], _msg_subs[1]))));
//      boost::get<typename Sync<ApproxSync2>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, MPtr(), MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
//      break;

//    case 3:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync3>::Ptr(new typename Sync<ApproxSync3>::type(ApproxSync3(q_size_),
//                                                                                                   _msg_subs[0], _msg_subs[1], _msg_subs[2]))));
//      boost::get<typename Sync<ApproxSync3>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, MPtr(), MPtr(), MPtr(), MPtr(), MPtr()));
//      break;

//    case 4:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync4>::Ptr(new typename Sync<ApproxSync4>::type(ApproxSync4(q_size_),
//                                                                                     _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3]))));
//      boost::get<typename Sync<ApproxSync4>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, MPtr(), MPtr(), MPtr(), MPtr()));
//    break;

//    case 5:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync5>::Ptr(new typename Sync<ApproxSync5>::type(ApproxSync5(q_size_),
//                                                                       _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4]))));
//      boost::get<typename Sync<ApproxSync5>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, MPtr(), MPtr(), MPtr()));
//      break;

//    case 6:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync6>::Ptr(new typename Sync<ApproxSync6>::type(ApproxSync6(q_size_),
//                                                         _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5]))));
//      boost::get<typename Sync<ApproxSync6>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, MPtr(), MPtr()));
//      break;

//    case 7:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync7>::Ptr(new typename Sync<ApproxSync7>::type(ApproxSync7(q_size_),
//                                           _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5], _msg_subs[6]))));
//      boost::get<typename Sync<ApproxSync7>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, MPtr()));
//      break;

//    case 8:
//      _approx_synchronizer.reset(new VariantApproxSync(typename Sync<ApproxSync8>::Ptr(new typename Sync<ApproxSync8>::type(ApproxSync8(q_size_),
//                             _msg_subs[0], _msg_subs[1], _msg_subs[2], _msg_subs[3], _msg_subs[4], _msg_subs[5], _msg_subs[6], _msg_subs[7]))));
//      boost::get<typename Sync<ApproxSync8>::Ptr>(*_approx_synchronizer)->registerCallback(
//                    boost::bind(&SyncImplHandler<M>::wrapCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
//      break;
//    }
  return true;
}

#endif // ROS_IMG_SYNC_SYNC_IMPL_HANDLER_H
