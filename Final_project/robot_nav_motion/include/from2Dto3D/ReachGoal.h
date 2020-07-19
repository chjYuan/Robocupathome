// Generated by gencpp from file from2Dto3D/ReachGoal.msg
// DO NOT EDIT!


#ifndef FROM2DTO3D_MESSAGE_REACHGOAL_H
#define FROM2DTO3D_MESSAGE_REACHGOAL_H

#include <ros/service_traits.h>


#include <from2Dto3D/ReachGoalRequest.h>
#include <from2Dto3D/ReachGoalResponse.h>


namespace from2Dto3D
{

struct ReachGoal
{

typedef ReachGoalRequest Request;
typedef ReachGoalResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ReachGoal
} // namespace from2Dto3D


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::from2Dto3D::ReachGoal > {
  static const char* value()
  {
    return "172c1930e2eae19890d6e9b43691f21a";
  }

  static const char* value(const ::from2Dto3D::ReachGoal&) { return value(); }
};

template<>
struct DataType< ::from2Dto3D::ReachGoal > {
  static const char* value()
  {
    return "from2Dto3D/ReachGoal";
  }

  static const char* value(const ::from2Dto3D::ReachGoal&) { return value(); }
};


// service_traits::MD5Sum< ::from2Dto3D::ReachGoalRequest> should match 
// service_traits::MD5Sum< ::from2Dto3D::ReachGoal > 
template<>
struct MD5Sum< ::from2Dto3D::ReachGoalRequest>
{
  static const char* value()
  {
    return MD5Sum< ::from2Dto3D::ReachGoal >::value();
  }
  static const char* value(const ::from2Dto3D::ReachGoalRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::from2Dto3D::ReachGoalRequest> should match 
// service_traits::DataType< ::from2Dto3D::ReachGoal > 
template<>
struct DataType< ::from2Dto3D::ReachGoalRequest>
{
  static const char* value()
  {
    return DataType< ::from2Dto3D::ReachGoal >::value();
  }
  static const char* value(const ::from2Dto3D::ReachGoalRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::from2Dto3D::ReachGoalResponse> should match 
// service_traits::MD5Sum< ::from2Dto3D::ReachGoal > 
template<>
struct MD5Sum< ::from2Dto3D::ReachGoalResponse>
{
  static const char* value()
  {
    return MD5Sum< ::from2Dto3D::ReachGoal >::value();
  }
  static const char* value(const ::from2Dto3D::ReachGoalResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::from2Dto3D::ReachGoalResponse> should match 
// service_traits::DataType< ::from2Dto3D::ReachGoal > 
template<>
struct DataType< ::from2Dto3D::ReachGoalResponse>
{
  static const char* value()
  {
    return DataType< ::from2Dto3D::ReachGoal >::value();
  }
  static const char* value(const ::from2Dto3D::ReachGoalResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // FROM2DTO3D_MESSAGE_REACHGOAL_H
