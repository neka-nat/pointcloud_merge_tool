# pointcloud merge tool

## Installation

```
curl -sSL https://raw.githubusercontent.com/python-poetry/poetry/master/get-poetry.py | python3
cd pointcloud_merge_tool
poetry install
```

## Run script

```
python pointcloud_merge_tool.py data/2021-08-29-01-35-21_2.bag --source /lidars/livox_avia_1/point_cloud2 --target /lidars/livox_avia_2/point_cloud2
```