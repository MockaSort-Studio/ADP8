# AI Folder

This folder contains AI-related code and examples for the SpotMicro project.

## Running the Example

To run the provided example, use the following command:

```sh
bazel run //ai:main -- --accelerator=cpu --strategy=ddp --devices=2
```

## Visualizing TensorBoard Logs

To visualize the TensorBoard logs, run:

```sh
bazel run //ai/utils:tensorboard -- --logdir absolute_path_to_fabric_logs
```

## About the Example

The example is adapted from [this PyTorch Lightning Fabric reinforcement learning example](https://github.com/Lightning-AI/pytorch-lightning/blob/master/examples/fabric/reinforcement_learning/).  
It has been modified to work with Bazel. Further deep adaptation is required for full integration with the SpotMicro project.
