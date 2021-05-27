# Xplane-screen-capture

Source code for X-Plane plugin that enables the user to export visualisation and depth buffer data from X-Plane to MATLAB using the Boost shared memory interface.

Griffith, E. J., Mishra, C., Ralph, J. F., & Maskell, S. (2018). A system for the generation of synthetic Wide Area Aerial surveillance imagery. *Simulation Modelling Practice and Theory*, *84*, 286-308.

```bibtex
@article{griffith2018system,
  title={A system for the generation of synthetic Wide Area Aerial surveillance imagery},
  author={Griffith, Elias J and Mishra, Chinmaya and Ralph, Jason F and Maskell, Simon},
  journal={Simulation Modelling Practice and Theory},
  volume={84},
  pages={286--308},
  year={2018},
  publisher={Elsevier}
}
```

### Pre-requisite
- Microsoft Visual Studio (Tested on v2017 (v15.9.7))
- Boost Libraries - https://www.boost.org/ (Tested on v1.70)
- MATLAB (Tested on v2018b)
- X-Plane (Tested on v10.51r2)
- Windows (Tested on Windows 10 - user can port to Linux/Mac)

### Setup
Set environment variables:

- BOOST_INC: Path to the Boost include files
- BOOST_LIB: Path to the Boost libraries 
- XPLANE_ROOT: Path to X-Plane root directory

### License
Copyright &copy; 2012, http://code.google.com/p/example-xplane-plugins/
Copyright &copy; 2021, The University of Liverpool

###### Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of "example-xplane-plugins" nor "The University of Liverpool" nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL "example-xplane-plugins" or "The University of Liverpool" BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



