mv ./cpp/build ../
mv ./multi_thread_cpp/build ../
git add .
git commit -m "update"
git push origin master
mv ../build ./cpp/
mv ../build ./multi_thread_cpp
