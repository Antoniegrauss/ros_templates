# Recursive service

Calling a service inside a service in python.
This requires the top-level service to:

- Run inside a MultiThreadedExecutor
- Execute the service inside a ReentrantCallbackGroup

The service callback has to be exited to execute the nested
service. Then re-entered to send the response.

Also note that the `spin_until_future_complete` call has to be
called on the `executor`. So the executor has to be passed to
the node as argument.

## Order of execution

- `outer_client` sends a request to `outer_server`
    - `outer_server` sends receives the request
    - `outer_server` sends a request to `inner_server`
        - `inner_server` receives the request and sends a response
    - `outer_server` receives the response ands sends a response
- `outer_client` receives the response
