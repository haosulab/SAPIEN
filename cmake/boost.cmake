if(TARGET boost_iostreams)
    return()
endif()

include(FetchContent)
FetchContent_Declare(
    boost
    URL      https://github.com/boostorg/boost/releases/download/boost-1.81.0/boost-1.81.0.tar.gz
    URL_HASH MD5=ffac94fbdd92d6bc70a897052022eeba
    OVERRIDE_FIND_PACKAGE
)

FetchContent_MakeAvailable(boost)
set_target_properties(boost_filesystem PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_serialization PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_system PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
set_target_properties(boost_thread PROPERTIES POSITION_INDEPENDENT_CODE TRUE)
