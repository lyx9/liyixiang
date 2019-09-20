#! /bin/bash

PB_INPUT_DIR=pb
PYTHON_OUTPUT_DIR=out
function my_make() {
if [ -d ${PB_INPUT_DIR} ]; then
    rm -rf ${PB_INPUT_DIR}
fi
if [ -d ${PYTHON_OUTPUT_DIR} ]; then
    rm -rf ${PYTHON_OUTPUT_DIR}
fi

mkdir ${PB_INPUT_DIR}
mkdir ${PYTHON_OUTPUT_DIR}

current_pwd=`pwd`
adu_dir="${current_pwd}/../../../../baidu/adu"
adu_3rd_dir="${current_pwd}/../../../../baidu/adu-3rd"
PROTOBUF=${adu_3rd_dir}/protobuf
LD_LIBRARY_PATH=${adu_3rd_dir}/protobuf/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH

COMMON_DIR=${adu_dir}/common
if [ ! -d  ${COMMON_DIR} ]; then
    git clone --depth 1 ssh://icode.baidu.com:8235/baidu/adu/common ${COMMON_DIR}
fi
cp ${COMMON_DIR}/proto/*/*.proto ${PB_INPUT_DIR}

if [ ! -d ${PROTOBUF} ]; then
    git clone --depth 1 ssh://icode.baidu.com:8235/baidu/adu-3rd/protobuf ${PROTOBUF}
fi

export PATH=${PROTOBUF}/bin:$PATH

protoc -I=${PB_INPUT_DIR} --python_out=${PYTHON_OUTPUT_DIR} ${PB_INPUT_DIR}/*.proto && echo "" > ${PYTHON_OUTPUT_DIR}/__init__.py

}

function my_clean() {
    rm -rf ${PB_INPUT_DIR} ${PYTHON_OUTPUT_DIR}
    echo "cleaned ok."
}

case $1 in
clean)
    my_clean
    ;;
*)
    my_make
    ;;
esac
