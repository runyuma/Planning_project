# To run our code
Our code is based on the [gym_envs_urdf](https://github.com/maxspahn/gym_envs_urdf) simulation environments and the [acados](https://docs.acados.org/) package. Please first install these packages before running our code.

### Install urdf_gym environment

To use the urdf gym environment, please first download the repository.

```
git clone git@github.com:maxspahn/gym_envs_urdf.git
```

Hereby, we recommend to create a conda environment and activate it before moving onto next step. We will take the python 3.8 version as an example:

```
conda create --name <env_name> python=3.8
soure activate <env_name> 
```

Then you can install the package using pip as:

```
pip install .
```

To check whether you have successfully install the simulation environment, you can run the following line inside the terminal.

```
python point_robot.py
```

##Install acados package

###Install acados

We here assume you are using the Linux system and you have cmake installed on your system. Please first clone acados and its submodules by running:

```
git clone git@github.com:acados/acados.git
cd acados
git submodule update --recursive --init
```
Install **acados** as follows:
```
mkdir -p build
cd build
cmake -DACADOS_WITH_QPOASES=ON ..
make install -j4
```
###Install acados python interface

Please first enter the virtual environment created previously,
```
conda activate <env_name>
```
Install ```acados_template``` Python package:
```
pip install -e <acados_root>/interfaces/acados_template
```
Add the path to the complied shared libraries by adding these two lines to the ~/.bashrc.

```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:"<acados_root>/lib"
export ACADOS_SOURCE_DIR="<acados_root>"
```
Till now, you can then enter the directory `Planning_project` and run the code under the directory `scenarios\parking_tasks`, for example:

```
python complex.py
```
