## 1 - CUDA and NVIDIA driver install according to your GPU

Please find GPU model:
```shell
lspci | grep ' VGA '
```
From your GPU model, find its computing capability, which driver your GPU can use. You can find this information on https://developer.nvidia.com/cuda-gpus (there is also a table at https://en.wikipedia.org/wiki/CUDA). We prefer to install CUDA and NVIDIA GPU driver at the same time. If you just want to install the driver, and cuda separately you can follow https://linuxconfig.org/how-to-install-the-latest-nvidia-drivers-on-ubuntu-16-04-xenial-xerus .
 
We will install CUDA 10.0 from NVIDIA. The package needs to be download from https://developer.nvidia.com/cuda-10.0-download-archive
Select the *deb network* version and download the *.deb* file. Go to the folder where the .deb file was downloaded and add exection permsision to it:
```shell
cd ~/Downloads
chmod +x cuda-repo-ubuntu1604_10.0.130-1_amd64.deb
```
Then follow the instructions given on https://developer.nvidia.com/cuda-10.0-download-archive with the installer
```shell
sudo dpkg -i cuda-repo-ubuntu1604_10.0.130-1_amd64.deb
sudo apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/7fa2af80.pub
sudo apt update
apt-cache show cuda-10-0
```
Now select the package version and install CUDA
```shell
sudo apt install cuda-10-0=10.0.130-1
echo "export PATH=/usr/local/cuda-10.0/bin:/usr/local/cuda-10.0/NsightCompute-1.0${PATH:+:${PATH}}" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=/usr/local/cuda-10.0/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> ~/.bashrc
```
Reboot the PC and do sanity check:
```shell
nvidia-smi
nvcc -V
```

## 2 - Install CUDNN according to your GPU, CUDA and NVIDIA driver versions

In order to select the CUDNN package version that fits to your GPU model, look for the configuration of requirements at https://docs.nvidia.com/deeplearning/sdk/cudnn-support-matrix/index.html
For example a GPU model *Quadro M2000M*, has compute capability of 5.0 (Maxwell) and is suported by *libcudnn7.6.4*. So we now just need to install the version compatible with Cuda 10.0 with this model
```shell
apt-cache show libcudnn7-dev
```
And then we select the good package version that is compatible with CUDA 10 (7.6.4.38-1+cuda10.0)
```shell
sudo apt install libcudnn7-dev=7.6.4.38-1+cuda10.0
```

## 3 - Full installation sanity check

In order to test if you have your NVIDIA driver, CUDA and CUDNN properly installed, you can test all with TensorFlow in a single command. To install tensorflow:
```shell
sudo apt install python-pip
pip install tensorflow-gpu==1.14.0
```
And then:
```shell
python -c "import tensorflow as tf; print(tf.test.is_gpu_available())"
```
If everything is ok you will have a *True* printed at the end of the messages. 

*OBS:* after the sanity check you can remove tensorflow if the library is not useful to your projects with `pip uninstall tensorflow-gpu` 
