
if [ $# == 2 ]; then
    if [ "$1" == "-a"]; then
        echo cleaning all
        rm -r Thirdparty/DBoW2/build  2> /dev/null
        rm -r Thirdparty/g2o/build  2> /dev/null
        rm -r Thirdparty/Sophus/build  2> /dev/null
        rm Vocabulary/ORBvoc.txt  2> /dev/null
    else
        echo cleaning primary
    fi
else
    echo cleaning primary
fi

rm -r build  2> /dev/null

echo cleaning complete