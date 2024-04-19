# Adds the test source files to CMake so that CLion recognises them properly.
# The test file itself may still say that it does not belong to any project target, but Intellisense should work as
# expected which is everything that is required.
# Trivia fact: Supposedly this fix is not needed as of a year ago, but in my case it was needed. It may be a good idea
# to periodically remove it and see if the issue is fixed
FILE(GLOB_RECURSE TEST_LIST
        ${CMAKE_CURRENT_LIST_DIR}/test/*.*
)
list(APPEND SRC_LIST ${TEST_LIST})
