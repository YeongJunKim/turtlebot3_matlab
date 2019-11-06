function deleteTimer()
tmrList = timerfind();
delete(tmrList);
rosshutdown
end