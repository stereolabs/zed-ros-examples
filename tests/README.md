# Test nodes
This folder contains a series of test nodes to verify the correctness of some behaviours of the ZED Wrapper nodes and nodelets. 
All the tests can be used as an example to create new nodes with similar features

- [RGB/Depth sync test](./zed_sync_test) : `zed_sync_test` - this is a nodelet to test that RGB and Depth topics with the same timestamp are correctly synchronized. The nodelets subscribes to RGB and depth topics using `message_filters` and publishes a new merged image to verify that depth and RGB are correctly overlapped.
