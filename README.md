# A Parametric-Adaptive Filter for Improving LiDAR Performance Under Snowfall Conditions

## 📖 Introduction
This study proposes a probabilistic model-driven adaptive filter to effectively remove the snowfall noise in the LiDAR point clouds and improve the detection capability of LiDAR under snowfall conditions. Firstly, the proposed filter extracts the snow noise from the point cloud, establishes the spatial distribution model of the snow noise, and obtains the key parameters of the model. Then, a density filter based on the spatial model is designed to extract the snow noise, and the spatial model and filter parameters are iteratively optimized.  Compared with the existing methods, the filter has better scene adaptability and low dependence on the priori datasets, and can effectively remove the snow noise from different types of LiDAR sensors under varying snowfall intensities. 

---

## 🛠️ Algorithmic operating environment
### Software Requirements
- **operating system**: Windows 10+/Linux Ubuntu 18.04+
- **MATLAB version**: R2022a or higher version
- **Necessary toolbox**: Point Cloud Processing Toolbox

## 🚀 Quick Start
1. The benchmark comparison algorithms are chosen as ROR [1], SOR [1], DROR [2], DSOR [3], DDIOR [4], and the implementation of the algorithms is in the BenchFilters.m file. Each benchmark algorithm is encapsulated as a function.The call is made by method name + parameters, and the return value is the filtered noise and non-noise. For example, [NOISE,NON_NOISE] = ROR(POINTCLOUD,SEARCH_RADIUS,MIN_POINTS)

2. Same for PMDF filters.

[1]	R. B. Rusu and S. Cousins, “3d is here: Point cloud library (pcl),” in 2011 IEEE international conference on robotics and automation, IEEE, 2011, pp. 1–4. Accessed: Jul. 28, 2024. [Online]. Available: https://ieeexplore.ieee.org/abstract/document/5980567/
[2]	N. Charron, S. Phillips, and S. L. Waslander, “De-noising of lidar point clouds corrupted by snowfall,” in 2018 15th Conference on Computer and Robot Vision (CRV), IEEE, 2018, pp. 254–261. Accessed: Jul. 13, 2024. [Online]. Available: https://ieeexplore.ieee.org/abstract/document/8575761/
[3]	A. Kurup and J. Bos, “DSOR: A Scalable Statistical Filter for Removing Falling Snow from LiDAR Point Clouds in Severe Winter Weather,” Oct. 30, 2021, arXiv: arXiv:2109.07078. Accessed: Jul. 13, 2024. [Online]. Available: http://arxiv.org/abs/2109.07078
[4]	W. Wang, X. You, L. Chen, J. Tian, F. Tang, and L. Zhang, “A scalable and accurate de-snowing algorithm for LiDAR point clouds in winter,” Remote Sensing, vol. 14, no. 6, p. 1468, 2022.
## 🧠 PMDF-Core Algorithm Analysis
    ```matlab
	% 1. Converts a 2D voxel to a 1D linearly indexed Cell array 
	sim_grid_flat = cell(z_find * r_find, 1);
	grid_six_flat = cell(z_find * r_find, 1);
	for i = 1:z_find
		for j = 1:r_find
			idx = (i-1)*r_find + j;
			sim_grid_flat{idx} = sim_grid{j,i};
			grid_six_flat{idx} = grid_six{j,i};
		end
	end

	% 2. Pre-calculated effective index and radius 
	valid_idx = zeros(z_find*r_find,1);
	radii_values = zeros(z_find*r_find,1);
	valid_count = 1;
	for i = 1:z_find
		for j = 1:r_find
			if ~isempty(sim_grid{j,i}) && ~isempty(grid_six{j,i})
				idx = (i-1)*r_find + j;
				valid_idx(valid_count,1) = idx;
				% if j*dr < 1
				   % radii_values(valid_count,1) = 0.12; 
				% else
					radii_values(valid_count,1) = 0.024 * j * dr;
				% end
				valid_count = valid_count + 1;
			end
		end
	end
	function [gamma_fit,k_gamma,theta_gamma] = DisFit(points,distribution)

		% Assume data(:,1) and data(:,2) are x and y coordinates
		xy = points(:,1:2);  % Extract x and y coordinates

		% Project points to x-y plane and calculate distance from each point to origin
		distances = sqrt(xy(:,1).^2 + xy(:,2).^2);

		% Fit Gamma distribution using fitdist
		gamma_fit = fitdist(distances, distribution);

		% Get parameters of Gamma distribution
		k_gamma = gamma_fit.a;  % Shape parameter
		theta_gamma = gamma_fit.b;  % Scale parameter
	end

	function [tlc_fit,mu_t,sigma_t,nu_t] = HeiFit(points, distribution)

		% Assume data(:,1) and data(:,2) are x and y coordinates
		z = points(:,3);  % Extract z coordinate

		% Project points to x-y plane and calculate distance from each point to origin
		tlc_fit = fitdist(z, distribution);
		mu_t = tlc_fit.mu;  % Location parameter
		sigma_t = tlc_fit.sigma;  % Scale parameter
		nu_t = tlc_fit.nu;  % Degrees of freedom parameter
	end

	function points = generate_point_cloud(N, k_gamma, theta_gamma, mu_t, sigma_t, nu_t)
		% Generate 3D point cloud data following specific distributions
		% Input:
		%   N            - Total number of points
		%   k_gamma      - Shape parameter of Gamma distribution
		%   theta_gamma  - Scale parameter of Gamma distribution
		%   mu_t         - Location parameter of t-location-scale distribution
		%   sigma_t      - Scale parameter of t-location-scale distribution
		%   nu_t         - Degrees of freedom parameter of t-location-scale distribution
		% Output:
		%   points       - N x 3 point cloud data (x, y, z coordinates)

		% 1. Generate points on x-y plane with distances following Gamma distribution
		distances = gamrnd(k_gamma, theta_gamma, N, 1);  % Generate N distances from Gamma distribution

		% 2. Generate azimuth angles uniformly distributed in [0, 2*pi]
		angles = 2 * pi * rand(N, 1);

		% 3. Generate heights following t-location-scale distribution
		heights = mu_t + sigma_t * trnd(nu_t, N, 1);  % Generate N samples from t distribution

		% 4. Convert polar coordinates to 3D Cartesian coordinates
		% x = r * cos(θ)
		% y = r * sin(θ)
		% z = z
		noise_std = 0.1;
		x = distances .* cos(angles);
		y = distances .* sin(angles);
		z = heights;
		x = x + noise_std * randn(N, 1);
		y = y + noise_std * randn(N, 1);
		z = z + noise_std * randn(N, 1);

		% 5. Combine into point cloud data
		points = [x, y, z];
	end
    ```
## 📊 Performance Evaluation
| Datasets | Filters | Presicion | Recall | F1-Score | Average Time(ms) |
| :---     | :----:  | :----:    | :----: | :----:   |----:            |
| CADC | ROR | 22.94 | 74.9 | 35.1 | 142 |
| CADC | DROR | 87.1 | 92.7 | 89.8 | 265 |
| CADC | SOR | 25.11 | 66.2 | 36.4 | 118 |
| CADC | DSOR | 90.1 | 92.1 | 91.1 | 215 |
| CADC | DDIOR | 89.9 | 90.5 | 90.2 | 139 |
| CADC | PMDF | 89.3 | 96.5 | 92.9 | 45 |
| CADC | ROR | 12.8 | 67.4 | 21.5 | 213 |
| CADC | DROR | 88.4 | 77.9 | 82.8 | 421 |
| CADC | SOR | 14.6 | 73.3 | 24.4 | 192 |
| CADC | DSOR | 90.1 | 82.1 | 85.9 | 267 |
| CADC | DDIOR | 79.1 | 75.3 | 77.2 | 152 |
| CADC | PMDF | 87.2 | 92.6 | 90.9 | 67 |
| CADC | ROR | 10.1 | 39.8 | 16.1 | 176 |
| CADC | DROR | 93.1 | 49.1 | 64.3 | 351 |
| CADC | SOR | 12.5 | 44.7 | 19.5 | 167 |
| CADC | DSOR | 89.2 | 87.6 | 88.3 | 241 |
| CADC | DDIOR | 88.2 | 85.8 | 86.9 | 163 |
| CADC | PMDF | 86.4 | 90.2 | 90.6 | 48 |
## 🔮 Future Plans
- [ ] Release of optimized C++ version 
- [ ] Real-time processing module based on ROS system 
## ❓ Support
Please submit an issue or contact if you have questions: 📧 chsun@chd.edu.cn
