#! /bin/sh
#
# This dot-script can be sourced for environment variables describing this
# project.
#
# Author: Kaj Munhoz Arfvidsson


################################################################################
################################################################################

##
## https://gist.github.com/kaarmu/123f536abd46fc86b0d720c8137a13a7
##


# Print error and exit
# > panic [CODE] MESSAGE
panic() {
    [ $# -gt 1 ] && CODE=$1 && shift || CODE=1
    echo "Error ($CODE): $1"
    usage
    exit $CODE
}

# Assert there exist commands
# > assert_command [COMMANDS...]
assert_command() {
    for cmd in "$@"; do
        [ ! "$(command -v "$cmd")" ] && panic "Missing command \"$cmd\". Is it installed?"
    done
}

# Return the argument at index
# > index [-]NUM [ARGS...]
index() {
    [ $# -lt 2 ] && return 1
    [ $1 -gt 0 ] && i=$1 || i=$(($# + $1))
    [ $i -le 0 ] && echo "" || ( shift $i && echo "$1" )
}

# Replace first substring with something else
# > replace SUB NEW TEXT
replace() {
    case "$3" in
        *"$1"* ) echo "${3%%$1*}$2${3#*$1}" ;;
        "$3" ) echo "$3" ;;
    esac
}

# Climb directories to find an existing path CHILD
# > climb CHILD [ PARENT ]
climb() {
    CHILD="$1"
    PARENT="$(realpath "${2:-$PWD}")"
    [ -e "$PARENT/$CHILD" ] && echo "$PARENT" && return 0
    [ "$PARENT" = "/" ] && return 1
    climb "$CHILD" "$PARENT/.."
    return $?
}


################################################################################
################################################################################

# # Enable to see debug information
# UTIL_DEBUG=1

REPOSITORY_PATH="$(climb .git)"
REPOSITORY_NAME="$(basename "$REPOSITORY_PATH")"

BUILD_CONTEXT="$REPOSITORY_PATH"
IMAGE_TAG="$(basename "$BUILD_CONTEXT")"
IMAGE_TAG="${IMAGE_TAG%%.*}"
CONTAINER_NAME="$REPOSITORY_NAME"

ROSDISTRO="melodic"
WORKSPACE="/$REPOSITORY_NAME"

if [ "$UTIL_DEBUG" ]; then
    echo "# Setting environment variables..."
    echo UTIL_DEBUG="$UTIL_DEBUG"
    echo REPOSITORY_PATH="$REPOSITORY_PATH"
    echo REPOSITORY_NAME="$REPOSITORY_NAME"
    echo BUILD_CONTEXT="$BUILD_CONTEXT"
    echo IMAGE_TAG="$IMAGE_TAG"
    echo CONTAINER_NAME="$CONTAINER_NAME"
    echo ROSDISTRO="$ROSDISTRO"
    echo WORKSPACE="$WORKSPACE"
fi

export UTIL_DEBUG
export REPOSITORY_PATH
export REPOSITORY_NAME
export BUILD_CONTEXT
export IMAGE_TAG
export CONTAINER_NAME
export ROSDISTRO
export WORKSPACE
