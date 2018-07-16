from math import sqrt

class Point:
	def __init__(self, x,y,z):
		self.x = x
		self.y = y
		self.z = z

	def __repr__(self):
		return "("+str(self.x)+","+str(self.y)+","+str(self.z)+")"
	
	def __sub__(self, other):
		return Point(self.x - other.x, self.y - other.y, self.z - other.z)
	
	def __add__(self, other):
		return Point(self.x + other.x, self.y + other.y, self.z + other.z)

	def dist(self, other):
		return sqrt( (self.x - other.x) ** 2 + (self.y - other.y) ** 2 + (self.z - other.z) ** 2)
		


# L3D.py (module 'L3D') Version 1.05
## Copyright (c) 2006 Bruce Vaughan, BV Detailing & Design, Inc.
## All rights reserved.
## NOT FOR SALE. The software is provided "as is" without any warranty.
################################################################################

################################################################################
"""
	LineLineIntersect3D (class) - Determine information about the intersection of two line segments in 3D space
	DistancePointLine3D (class) - Determine information about the relationship between a line segment and a point in 3D space
	ret_WP (class) - Return the WP on member 1 and which end of member 2 coincides

	Revision History:
		Version 1.02 - Add attributes obj.Pa and obj.Pb to a DistancePointLine3D instance
		Version 1.03 (10/30/06) - Rework calculation of self.position
								  Consolidate comments
								  add function ret_WP
		Version 1.04 (11/16/06) - Rework LineLineIntersect3D - solve using like triangles
		Version 1.05 (11/17/06) - Rework LineLineIntersect3D - solve for unknowns by substitution

	Reference 'The Shortest Line Between Two Lines in 3D' - Paul Bourke   
"""
class LineLineIntersect3D:
	
	def __init__(self, p1, p2, p3, p4):
		"""																													   <-->	 <-->
			Calculate the points in 3D space Pa and Pb that define the line segment which is the shortest route between two lines p1p2 and p3p4.
			Each point occurs at the apparent intersection of the 3D lines.
			The apparent intersection is defined here as the location where the two lines 'appear' to intersect when viewed along the line segment PaPb.
			Equation for each line:
			Pa = p1 + ma(p2-p1)
			Pb = p3 + mb(p4-p3)
			
			Pa lies on the line connecting p1p2.
			Pb lies on the line connecting p3p4.

			The shortest line segment is perpendicular to both lines. Therefore:
			(Pa-Pb).(p2-p1) = 0
			(Pa-Pb).(p4-p3) = 0

			Where:			
			'.' indicates the dot product			

			A = p1-p3
			B = p2-p1
			C = p4-p3

			Substituting:
			(A + ma(B) - mb(C)).B = 0	   &	   (A + ma(B) - mb(C)).C = 0
			-----------------------------------------------------------------
			A.B + ma(B.B) - mb(C.B) = 0
			A.B + ma(B.B) - (ma(C.B)-A.C)/C.C)(C.B) = 0
			ma(B.B)(C.C) - ma(C.B)(C.B) = (A.C)(C.B)-(A.B)(C.C)
			ma = ((A.C)(C.B)-(A.B)(C.C))/((B.B)(C.C) - (C.B)(C.B))
			mb = (A.B + ma(B.B))/(C.B)

			If the cross product magnitude of the two lines is equal to 0.0, the lines are parallel.		  
																																				 <-->
			A line extends forever in both directions. The name of a line passing through two different points p1 and p2 would be "line p1p2" or p1p2.										   
			The two-headed arrow over p1p2 signifies a line passing through points p1 and p2.

			Two lines which have no actual intersection but are not parallel are called 'skew' or 'agonic' lines. Skew lines can only exist in
			three or more dimensions.

			Determine whether the apparent intersection point lies between the line segment end points or beyond one of the line segment end points.
			This information is to be used to evaluate the framing condition of mem1 (p1p2).
			Convention for members:
				p1p2 - mem1.left.location, mem1.right.location
				p3p4 - mem2.left.location, mem2.right.location
				
			Set a keyword indicating the apparent intersection point position with respect to the line segment end points p1 and p2 as follows:
				'LE' indicates the apparent intersection point occurs at p1 (within fudge_factor distance)
				'RE' indicates the apparent intersection point occurs at p2 (within fudge_factor distance)
				'Beyond LE' indicates the apparent intersection point occurs beyond p1
				'Beyond RE' indicates the apparent intersection point occurs beyond p2
				'Not Beyond LE' indicates the apparent intersection point occurs in between p1 and p2 and is closer to p1
				'Not Beyond RE' indicates the apparent intersection point occurs in between p1 and p2 and is closer to p2
			Calculate the magnitude and direction (beam member 'X' distance) the apparent intersection point occurs from line segment p1p2 end points.
		"""
		
		if 'tuple' == p1.__class__.__name__:
			p1 = Point(*p1)
			p2 = Point(*p2)
			p3 = Point(*p3)
			p4 = Point(*p4)
		
		def cross_product(p1, p2):
			return Point(p1.y*p2.z - p1.z*p2.y, p1.z*p2.x - p1.x*p2.z, p1.x*p2.y - p1.y*p2.x)

		def dot_product(p1, p2):
			return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z)

		def mag(p):
			return sqrt(p.x**2 + p.y**2 + p.z**2)		

		def normalise(p1, p2):
			p = p2 - p1
			m = mag(p)
			if m == 0:
				return Point(0.0, 0.0, 0.0)
			else:
				return Point(p.x/m, p.y/m, p.z/m)

		def ptFactor(p, f):
			return Point(p.x*f, p.y*f, p.z*f)

		A = p1-p3
		B = p2-p1
		C = p4-p3

		# Line p1p2 and p3p4 unit vectors
		self.uv1 = normalise(p1, p2)
		self.uv2 = normalise(p3, p4)		

		# Check for parallel lines
		self.cp12 = cross_product(self.uv1, self.uv2)
		self._cp12_ = mag(self.cp12)

		if round(self._cp12_, 6) != 0.0:		 
			ma = ((dot_product(A, C)*dot_product(C, B)) - (dot_product(A, B)*dot_product(C, C)))/ \
				 ((dot_product(B, B)*dot_product(C, C)) - (dot_product(C, B)*dot_product(C, B)))
			mb = (ma*dot_product(C, B) + dot_product(A, C))/ dot_product(C, C)
			
			# Calculate the point on line 1 that is the closest point to line 2
			Pa = p1 + ptFactor(B, ma)
			self.Pmem1 = Pa
			
			# Calculate the point on line 2 that is the closest point to line 1
			Pb = p3 + ptFactor(C, mb)
			self.Pmem2 = Pb
			
			# Distance between lines			
			self.inters_dist = Pa.dist(Pb)
			
			if round(ma, 3) >= 0.0 and round(ma, 3) <= 1.0:
				self.on_segment1 = 1
				xl_dir = 1
				xr_dir = -1
				if round(ma, 2) == 0.0:
					self.position = "LE" # apparent intersection is at p1
				elif round(ma, 2) == 1.0:
					self.position = "RE" # apparent intersection is at p2
					xr_dir = 1
					xl_dir = 1
				elif ma <= 0.5:
					self.position = "Not Beyond LE" # apparent intersection is closer to p1
				elif ma > 0.5:
					self.position = "Not Beyond RE" # apparent intersection is closer to p2
				else:
					print('self.position calculation error, self.on_segment = 1')
					raise ValueError
			else:
				self.on_segment1 = 0
				if ma < 0.0:
					self.position = "Beyond LE" # apparent intersection is beyond p1
					xl_dir = -1
					xr_dir = -1
				elif ma > 0.0:
					self.position = "Beyond RE" # apparent intersection is beyond p2
					xl_dir = 1
					xr_dir = 1
				else:
					print('self.position calculation error, self.on_segment = 0')
					raise ValueError

			# Set the member 'X' direction with respect to p1 and p2 - either '+' or '-'
			self.left_dist = round(Pa.dist(p1)*xl_dir, 8)
			self.right_dist = round(Pa.dist(p2)*xr_dir, 8)				

			if round(mb, 3) >= 0.0 and round(mb, 3) <= 1.0:
				self.on_segment2 = 1
			else:
				self.on_segment2 = 0
			
			# Calculate the unit vector of PaPb
			if round(self.inters_dist, 4) > 0.0:
				self.uv = normalise(Pb, Pa)
			else:
				self.uv = Point(0.0, 0.0, 0.0)
				
		# Lines are parallel
		else:
			self.Pmem1 = None
			self.Pmem2 = None
			self.inters_dist = None
			self.left_dist = None
			self.right_dist = None
			self.uv = None

	# Return False if lines are parallel, and return True if lines are not parallel		
	def not_parallel(self):
		if round(self._cp12_, 5) != 0.0:
			return True
		else:
			return False 
# end class definition


