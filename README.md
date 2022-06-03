# GTASynth: 3D synthetic data of outdoor non-urban environments
**GTAVSynth** is a synthetic dataset generated using Grand Theft Auto V (GTAV).


Direct URL to data: DO BE DONE

## Type of Data
<style type="text/css">
.tg  {border-collapse:collapse;border-color:#ccc;border-spacing:0;}
.tg td{background-color:#fff;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
  font-family:Arial, sans-serif;font-size:14px;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg th{background-color:#f0f0f0;border-color:#ccc;border-style:solid;border-width:1px;color:#333;
  font-family:Arial, sans-serif;font-size:14px;font-weight:normal;overflow:hidden;padding:10px 5px;word-break:normal;}
.tg .tg-0pky{border-color:inherit;text-align:left;vertical-align:top}
.tg .tg-0lax{text-align:left;vertical-align:top}
</style>
<table class="tg">
<thead>
  <tr>
    <th class="tg-0pky">Name</th>
    <th class="tg-0pky">Pairs</th>
    <th class="tg-0lax">Length (km)</th>
    <th class="tg-0lax">Loops</th>
  </tr>
</thead>
<tbody>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">river_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">1966</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">6.207</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">1</span></td>
  </tr>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">park_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">1516</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">2.873</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">6</span></td>
  </tr>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">mountain_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">2678</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">6.619</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">1</span></td>
  </tr>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">hill_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">3108</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">12.732</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">7</span></td>
  </tr>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">field_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">3252</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">7.356</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">8</span></td>
  </tr>
  <tr>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">canyon_sequence</span></td>
    <td class="tg-0pky"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">3687</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">8.871</span></td>
    <td class="tg-0lax"><span style="font-weight:400;font-style:normal;text-decoration:none;color:#000;background-color:transparent">1</span></td>
  </tr>
</tbody>
</table>

## Dataset Tools
### Python
* `create_360_from _velofile.py`: takes as input four raw point clouds to produce the 360° view.
* `create_map.py`: creates a map of the environment using the processed data.
* `point_cloud_player.py`: reproduce the point clouds in sequence, as they were acquired.
* `tools.py`: contains methods to read the data in the dataset.

### MATLAB
* `helperPointCloudPlayer.m`: the same as Point_cloud_player.py.
* `helperMapPathVisualizer.m`: with the path followed by the vehicle on the map.
* `helperFrameLocationOnMap.m`: view a single point cloud on the map.

## Related Article
