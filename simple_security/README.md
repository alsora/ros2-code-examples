# simple_security

Collection of scripts for testing Secure ROS2 (SROS2) features.


## How to use it

#### The pub/sub system

For our tests we will use a very simple pub/sub ROS2 system, made only of two nodes.
You can try them first, without security enabled.

    $ ros2 run simple_publisher publisher_main __node:=simple_publisher

    $ ros2 run simple_subscriber subscriber_main __node:=simple_subscriber

They are communicating on a topic called `my_topic`.
We are using the ROS2 CLI to overwrite the name of the nodes `__node:=<new_name>`.

If you want to check the source code of these nodes: [simple_publisher](../simple_publisher) and [simple_subscriber](../simple_subscriber).

#### Creating authentication keys

Then we can start enabling security controls for our ROS2 system.

The first thing to do is to create a keystore directory

    $ ros2 security create_keystore $HOME/simple_keys

Then we can populate the directory with authentication keys for our nodes.

    $ ros2 security create_key $HOME/simple_keys simple_publisher
    $ ros2 security create_key $HOME/simple_keys simple_subscriber

Now that we have defined some keys, it's necessary to enable the ROS2 security in all the terminals that we are using.

    $ source enable_security.sh

**NOTE**: you can reset these environment variables with `source disable_security.sh`.

Now security is enabled.

If you run again the previous two nodes, you will not see any change.
However, under the hood, messages are being encrypted.

If you try to create a node whose name has not any authentication key associated, you will get an error.

    $ ros2 run simple_publisher publisher_main __node:=additional_publisher

As long as security is enabled, you can only create nodes with authenticated names.

#### Setting access permissions

Once you have some nodes with an associated authentication key, it's also possible to refine their access permissions.

    $ ros2 security create_permission $HOME/simple_keys simple_publisher policies.yaml
    $ ros2 security create_permission $HOME/simple_keys simple_subscriber policies.yaml

The `policies.yaml` file contains specifications about which topics can be accessed (for publishing, subscribing or both) by each node.
In this case `simple_publisher` is allowed to publish on `my_topic` and `simple_subscriber` is allowed to subscribe to `my_topic`.

Thus, if you run again the initial two nodes, they will still work as expected.

If one of these nodes, for which we have set the access permissions, tries to register a publisher or a subscription for a different topic, you will get an error.

    $ ros2 run simple_publisher publisher_main __node:=additional_publisher __topic:=another_topic

**NOTE**: as of ROS2 Bouncy Bolson, it's possible to set access permissions only for topics. Support for services and actions will be implemented in the future.

#### Non-authenticated nodes

As we have seen, as long as security options are enabled, only nodes with autenticated names can be created.

If we have our secured pub/sub system running, we can still however start a new node without security options.

You can do that by opening a new terminal or with `source disable_security.sh`.

    $ ros2 run simple_subscriber subscriber_main __node:=additional_subscriber

You will get no error while creating this subscriber which do not has an authentication key. However, it will not receive any message from the publisher node running in the secured system.


#### Authenticated nodes without permissions

Let's now create authentication keys for an additional node.

    $ ros2 security create_key $HOME/simple_keys additional_subscriber

Now, we are able to run an additional node called `additional_subscriber` in our secured system.

    $ ros2 run simple_subscriber subscriber_main __node:=additional_subscriber

Note that this node does not have any access permissions policy set.

In this case the default behavior is that it has no restrictions, thus it will be able to receive messages.
However a security error is displayed.




