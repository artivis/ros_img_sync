/** \author Jeremie Deray. */

#ifndef _ROS_IMG_SYNC_MESSAGE_META_UTILS_H_
#define _ROS_IMG_SYNC_MESSAGE_META_UTILS_H_

#include <utility>
#include <type_traits>
#include <memory>
#include <vector>
#include <tuple>

namespace ros_msgs_sync{
namespace meta{

template <typename T>
using shared_pointer = std::shared_ptr<T>;

template <typename T, typename... Args>
shared_pointer<T> make_shared(Args&&... args)
{
  return std::make_shared<T>(std::forward<Args>(args)...);
}

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

//////////////////////////
// Helper to add shared_ptr to type //
//////////////////////////

template <typename T>
struct add_shared_ptr { using type = shared_pointer<T>; };

template <typename T>
struct add_shared_ptr< shared_pointer<T> > { using type = shared_pointer<T>; };

template <typename T>
using add_shared_ptr_t = typename add_shared_ptr<T>::type;

///////////////////////////////////////////
// Helper to remove shared_ptr from type //
///////////////////////////////////////////

template <typename T>
struct rm_shared_ptr { using type = T; };

template <typename T>
struct rm_shared_ptr< shared_pointer<T> > { using type = T; };

template <typename T>
using rm_shared_ptr_t = typename rm_shared_ptr<T>::type;

////////////////////////////////////////////////////
// Helper to add shared_ptr to each type in tuple //
////////////////////////////////////////////////////

template <typename T>
struct to_each_add_shared_ptr;

template <typename... Args>
struct to_each_add_shared_ptr<std::tuple<Args...>>
{
  using type = std::tuple<add_shared_ptr_t<Args>...>;
};

///////////////////////////////////
// Helper copy a vector to tuple //
///////////////////////////////////

template <typename T, std::size_t... Indices>
auto vectorToTupleHelper(const std::vector<T>& v, index_sequence<Indices...>)
-> decltype(std::make_tuple(v[Indices]...))
{
  return std::make_tuple(v[Indices]...);
}

template <std::size_t N, typename T>
auto vectorToTuple(const std::vector<T>& v)
-> decltype(vectorToTupleHelper(v, make_index_sequence<N>()))
{
  assert(v.size() >= N);
  return vectorToTupleHelper(v, make_index_sequence<N>());
}

///////////////////////////////////////////////////////////
// Helper to create a tuple with n args in variadic pack //
///////////////////////////////////////////////////////////

template <typename T, typename S>
struct subTuple;

template <template <std::size_t...> class S, std::size_t... Indices, typename... Args>
struct subTuple < std::tuple<Args...>, S<Indices...> >
{
    using type = decltype( std::make_tuple (std::get<Indices>( std::declval<std::tuple<Args...>>() )...) );
};

////////////////////////////
// Helper to concat tuple //
////////////////////////////

template<typename L, typename R>
struct tuple_cat;

template<typename... TL, typename... TR>
struct tuple_cat<std::tuple<TL...>, std::tuple<TR...>> {
    using type = std::tuple<TL..., TR...>;
};

template<typename L, typename R>
using tuple_cat_t = typename tuple_cat<L,R>::type;

//////////////////////////
// Helper to define a tuple of N type T //
//////////////////////////

//template <std::size_t I, typename T>
//struct toTupleNT_
//{
//  using type = tuple_cat_t<std::tuple<T>,typename toTupleNT_<I-1, T>::type>;
//};

//template <typename T> struct toTupleNT_<2,T> { using type = std::tuple<T,T>;};
//template <typename T> struct toTupleNT_<1,T> { using type = std::tuple<T>;};
//template <typename T> struct toTupleNT_<0,T> { using type = std::tuple<>;};

//template <std::size_t I, typename T>
//using TupleNT = typename toTupleNT_<I, T>::type;

//template <std::size_t I, template <typename...Args> class C, typename... Args>
//struct RepCont_
//{
//  using type = tuple_cat_t<C<Args...>,typename RepCont_<I-1, C, Args...>::type>;
//};

//template <template <typename...Args> class C,typename... Args> struct RepCont_<2,Args...> { using type = C<Args...,Args...>;};
//template <template <typename...Args> class C,typename... Args> struct RepCont_<1,Args...> { using type = C<Args...>;};
//template <template <typename...Args> class C,typename... Args> struct RepCont_<0,Args...> { using type = C<>;};

//template <std::size_t I, typename... Args>
//struct RepTup2_ : RepCont_<I, std::tuple<Args...>>
//{
//  using type;
//  //using type = tuple_cat_t<std::tuple<Args...>,typename RepTup_<I-1, Args...>::type>;
//};

template <std::size_t I, typename... Args>
struct RepTup_
{
  using type = tuple_cat_t<std::tuple<Args...>,typename RepTup_<I-1, Args...>::type>;
};

template <typename... Args> struct RepTup_<2,Args...> { using type = std::tuple<Args...,Args...>;};
template <typename... Args> struct RepTup_<1,Args...> { using type = std::tuple<Args...>;};
template <typename... Args> struct RepTup_<0,Args...> { using type = std::tuple<>;};

template <std::size_t I, typename ... Args>
using RepTup = typename RepTup_<I, Args...>::type;

//////////////////////////////
// Helper for_each in tuple //
//////////////////////////////

template<typename T, typename F, std::size_t... I>
void for_each(T&& t, F f, index_sequence<I...>)
{
  auto l = { (f(std::get<I>(std::forward<T>(t))), 0)... };
}

template<typename F, template <class...Ts> class C, typename... Ts>
void for_each(C<Ts...>&& t, F f)
{
  for_each(std::forward<C<Ts...>>(t), std::forward<F>(f), make_index_sequence<sizeof...(Ts)>());
}

} /* namespace meta */
} /* namespace ros_msgs_sync */

#endif /* _ROS_IMG_SYNC_MESSAGE_META_UTILS_H_ */
