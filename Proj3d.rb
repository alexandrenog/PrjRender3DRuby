require "Gosu"

include Math
class Point
  attr_accessor :x, :y, :z
	def initialize x=0.0, y=0.0, z=0.0
	  setP(x,y,z)
	end
	def print
	  puts "(#{x}, #{y}, #{z})"
	end
	def setP x,y,z
	  @x,@y,@z=x,y,z
	end
	def self.add p1, p2
    	Point.new(p1.x+p2.x,p1.y+p2.y,p1.z+p2.z)
	end
	def self.sub p1, p2
    	Point.new(p1.x-p2.x,p1.y-p2.y,p1.z-p2.z)
	end
	def self.mult p, m
    	Point.new(p.x*m, p.y*m, p.z*m)
	end
	def self.multVet p1, p2
	    i=p1.y*p2.z-p1.z*p2.y
	    j=p1.z*p2.x-p1.x*p2.z
	    k=p1.x*p2.y-p1.y*p2.x
	    return Point.new(i,j,k)
	end
	def self.multVetScalar p1, p2
	    return p1.x*p2.x+p1.y*p2.y+p1.z*p2.z
	end
	def rotZ teta
	  Point.new(x*cos(teta)-y*sin(teta), x*sin(teta)+y*cos(teta), z)
	end
	def rotX teta
	  Point.new(x, y*cos(teta)-z*sin(teta), y*sin(teta)+z*cos(teta))
	end
	def rotY teta
	  Point.new(x*cos(teta)+z*sin(teta), y, -x*sin(teta)+z*cos(teta))
	end
	def normal
	  (x**2.0+y**2.0+z**2.0)**0.5
	end
	def unitario
	  	norm = normal();
    	return Point.new(x/norm, y/norm, z/norm);
	end
	def to_a
		[x,y,z]
	end
	def self.spherical r, theta, phi
		Point.new(r*sin(theta)*cos(phi), r*sin(theta)*sin(phi), r*cos(theta))
	end
	def to_spherical 
		[normal,acos(z/normal),atan(y/x)]
	end
end
class Line
  attr_accessor :a, :b, :c, :xo, :yo, :zo
	def initialize p1, p2
    @xo,@yo,@zo=p1.x, p1.y, p1.z
    pop = Point::sub(p2,p1)
    @a,@b,@c=pop.x, pop.y, pop.z
  end
  def print t=nil
    if t !=nil
  		puts("Lx = %f + %f*t = %f" % [xo,a,xo+a*t]);
  		puts("Ly = %f + %f*t = %f" % [yo,b,yo+b*t]);
  		puts("Lz = %f + %f*t = %f" % [zo,c,zo+c*t]);
    else
  		puts("Lx = %f + %f*t" % [xo,a]);
  		puts("Ly = %f + %f*t" % [yo,b]);
  		puts("Lz = %f + %f*t" % [zo,c]);
    end
  end
  def point t
    Point.new(xo+a*t,yo+b*t,zo+c*t)
  end
end
class Plane
  attr_accessor :a, :b, :c, :d, :normal, :p1, :p2, :p3, :p4
	def initialize p1, p2, p3
	    @p1, @p2, @p3, @p4 = p1, p2, p3, Point::add(p3,Point::sub(p2,p1))
	    pop1, pop2 = Point::sub(@p2,@p1), Point::sub(@p3,@p1)
	    @normal=Point::multVet(pop1,pop2).unitario
	    @a,@b,@c=normal.x, normal.y, normal.z
	    @d=-a*@p1.x-b*@p1.y-c*@p1.z
  end
	def print
	  puts("%f*x + %f*y + %f*z + %f" % [a,b,c,d])
	end
  def check p
    return (a*p.x+b*p.y+c*p.z+d)
  end
end
def intersection(l, p) # line and plane
    t = -(p.a*l.xo+p.b*l.yo+p.c*l.zo+p.d)/(p.a*l.a+p.b*l.b+p.c*l.c)
    return l.point(t)
end
class Camera
    attr_accessor :projection_plane, :optic_center, :teta, :alfa, :sizex, :sizey, :inferior_edge
    def initialize(plane, plane_center, focal_distance, sizex, sizey)
        @projection_plane=plane
        @sizex, @sizey = sizex, sizey
        @optic_center=Point::sub(plane_center,Point::mult(plane.normal,focal_distance))
        #print("Centro Optico\n")
        #@optic_center.print
        @inferior_edge=Point::sub(projection_plane.p3,projection_plane.p1)
        left_edge=Point::sub(projection_plane.p2,projection_plane.p1)
        @teta=atan2(@inferior_edge.y,@inferior_edge.x)
        left_edge=left_edge.rotZ(-teta)
        @alfa=atan2(left_edge.z,left_edge.y)
        #puts("teta=%f, alfa=%f" % [teta, alfa])
    end
    def self.construct(center, direction, sizex, sizey)
        ort=direction.unitario()
        left=Point::mult(Point::multVet(ort,Point.new(0,0,1.0)),-1).unitario(); 
        vert=Point::multVet(left,ort).unitario();
        plane = Plane.new(	Point::add(Point::add(center, Point::mult(left,sizex/2)), Point::mult(vert,-sizey/2)),
            			Point::add(Point::add(center, Point::mult(left,sizex/2)), Point::mult(vert,+sizey/2)),
            			Point::add(Point::add(center, Point::mult(left,-sizex/2)), Point::mult(vert,-sizey/2)))
        #center.print()
        #Point::add(Point::add(center, Point::mult(left,-sizex/2)), Point::mult(vert,-sizey/2)).print()
        return Camera.new(plane,center,direction.normal(), sizex, sizey)
    end
    def proj p
        return intersection(Line.new(p,optic_center),projection_plane);
    end
    def planeToXY p
        ponto_plano, ponto_ref = p, projection_plane.p1
        ponto_dif=Point::sub(ponto_plano,ponto_ref)
        return ponto_dif.rotZ(-teta).rotX(-alfa)
    end
    def projXY p
        return planeToXY(proj(p))
    end
    
end
class Objeto
	attr_accessor :pontos, :faces, :posicao, :textura, :raio, :texturaunica
	def initialize 
		@pontos= @faces= @posicao = @textura = @texturaunica = nil
	end
	def rotaciona(x, y, z)
		@pontos = @pontos.map{|e|
	      	new_e=Point::sub(e,posicao)
		    new_e=new_e.rotX(x).rotY(y).rotZ(z)
	      	Point::add(posicao,new_e)
		}
	end
	def move_to pos
		dif=Point.sub(pos,@posicao)
		@pontos.each_with_index{|e,i|
			@pontos[i]=Point.add(e,dif)
		}
		@posicao=pos
	end
	def move_by posdif
		move_to(Point.add(@posicao,posdif))
	end
	def self.cubo posicao, dimensao, textura=nil
		cubo = Objeto.new
		cubo.texturaunica = true
		cubo.textura = textura
		cubo.raio=dimensao
	    cubo.posicao = posicao
	    cubo.pontos = Array.new(8){|i|
	    	Point::add(posicao,
	      	Point.new(dimensao*((i%2==0)?(-1):(1)),
	       			  dimensao*(((i/2)%2==0)?(-1):(1)),
	        		  dimensao*(((i/4)%2==0)?(-1):(1))))
	    }
	    cubo.faces=[[0,1,3,2],[4,5,7,6],[0,1,5,4],[2,3,7,6],[0,2,6,4],[1,3,7,5]].map{|e| {pontos: e}}
	    return cubo
	end
	def self.plataforma posicao, dimensao, linhas, colunas, textura=nil
		plataforma = Objeto.new
		plataforma.texturaunica = true
		plataforma.textura = textura
		#cubo.raio=dimensao
	    plataforma.posicao = posicao
	    plataforma.pontos = []
	    2.times{|k|
	    (linhas+1).times{|i|
	    (colunas+1).times{|j|
			plataforma.pontos<< Point::add(posicao,
						      	Point.new(dimensao*(j*2-1),
						       			  dimensao*(i*2-1),
						        		  dimensao*((k%2==0)?(-1):(1))))
	    }}}
	    plataforma.faces=[]
	    listep=(colunas+1)*(linhas+1) # level_idx_step
	    linhas.times{|i|
	    colunas.times{|j|
	    	plataforma.faces << {pontos: [j+i*(colunas+1), j+i*(colunas+1)+1, j+(i+1)*(colunas+1)+1, j+(i+1)*(colunas+1)]}
	    	plataforma.faces << {pontos: [j+i*colunas+listep, j+i*colunas+1+listep, j+(i+1)*colunas+2+listep, j+(i+1)*colunas+1+listep]}
	    }}
	    return plataforma
	end
	def self.piramide posicao, dimensao, textura=nil
		piramide = Objeto.new
		piramide.texturaunica = true
		piramide.textura = textura
		piramide.raio=dimensao
	    piramide.posicao = posicao
	    piramide.pontos = Array.new(4){|i|
	    	Point::add(posicao,
	      	Point.new(dimensao*((i%2==0)?(-1):(1)),
	       			  dimensao*(((i/2)%2==0)?(-1):(1)),
	        		 -dimensao))
	    }
	    piramide.pontos<<Point::add(posicao,Point.new(0,0,dimensao))
	    piramide.faces=[[0,1,4],[1,3,4],[3,2,4],[2,0,4],[0,1,3,2]].map{|e| {pontos: e}}
	    return piramide
	end
	def self.cruzado posicao, dimensao, textura=nil
		cruzado = Objeto.new
		cruzado.texturaunica = false
		cruzado.textura = [textura.subimage(0,0,textura.width/2,textura.height),textura.subimage(textura.width/2,0,textura.width/2,textura.height)]
		cruzado.raio=dimensao
	    cruzado.posicao = posicao
	    cruzado.pontos = Array.new(8){|i|
	    	Point::add(posicao,
	      	Point.new(dimensao*((i%2==0)?(-1):(1)),
	       			  dimensao*(((i/2)%2==0)?(-1):(1)),
	        		  dimensao*(((i/4)%2==0)?(-1):(1))))
	    }
	    cruzado.pontos<<Point::add(posicao,Point.new(0,0,dimensao))<<Point::add(posicao,Point.new(0,0,-dimensao))
	    cruzado.faces=[]
	    cruzado.faces<<{pontos: [4,8,9,0],textura: 0}
	    cruzado.faces<<{pontos: [8,7,3,9],textura: 1}
	    cruzado.faces<<{pontos: [5,8,9,1],textura: 0}
	    cruzado.faces<<{pontos: [8,6,2,9],textura: 1}
	    return cruzado
	end
end
class ProjWindow < Gosu::Window
  def initialize width, height, fullscreen=false
    super(width, height, fullscreen)
    #  Criação da Camera 
    @fov=84.0*(PI/180.0)
    @camAperture=0.01
    @camOrientacao = Point.spherical(fov_distance,-PI/4,PI/4)
    @camPos = Point.new(0,0,1)
    @cam= Camera::construct(@camPos,@camOrientacao,@camAperture*(width/height),@camAperture)
    @render_mode=0
    #  Criação dos Vertices do formato
    @objetos=[]
    @rock_texture=Gosu::Image.new(self, 'earth.png')
    @sand_texture=Gosu::Image.new(self, 'sand.png')
    @ncage_texture=Gosu::Image.new(self, 'ncage.bmp')
    @cane_texture=Gosu::Image.new(self, 'cane.bmp')
    l=20
    @objetos<<Objeto.plataforma(Point.new(-l/2,-l/2,0), 0.5, l, l, @rock_texture)
    l.times do |i|
    	l.times do |j|
	    	if rand(9)==0
		    	@objetos<<Objeto.cruzado(Point.new(i-l/2,j-l/2,1), 0.5, @ncage_texture)
		    end
    	end
    end
    #@objetos.each{|obj| obj.textura=[@earth_texture,@sand_texture,@ncage_texture].at(rand(3))}
  end
  def fov_distance
	@camAperture/tan(@fov/2)
  end
  def draw
  	render_objects_surfaces if [0,1,3].include? @render_mode
  	render_objects_mesh if [1,2,3].include? @render_mode
	Gosu::Image.from_text("r: %.2f, theta: %.2f, phi: %.2f" % (@camOrientacao.to_spherical.map{|e| e*180/PI}),26).draw(0,0,400)
	Gosu::Image.from_text("fov: %.2f, fov_distance: %.2f, camAperture: %.2f" % ([@fov*(180.0/PI), fov_distance, @camAperture]),26).draw(0,30,400)
  end
  def render_objects_mesh 
  	@objetos.each do |obj|
		objeto2d=proj3d(obj.pontos)
	  	obj.faces.each{|face|
	  		not_back = face[:pontos].map{|idx| obj.pontos[idx]}.inject(true){|acc,el|
		     	acc && (Point.multVetScalar(Point.sub(el,@cam.optic_center),Point.sub(@camPos,@cam.optic_center))>0)
		    }
		    (0...face[:pontos].size).each{|i|
		    	aux1=objeto2d[face[:pontos][i]]
		    	aux2=objeto2d[face[:pontos][(i+1)%face[:pontos].size]]
		    	aux1=Point.mult(aux1,width/(@cam.sizex))
		    	aux2=Point.mult(aux2,width/(@cam.sizex))
				draw_line(aux1.x,aux1.y,0xff0000ff,aux2.x,aux2.y,0xff0000ff) if not_back
			}
		}
	end
  end
  def render_objects_surfaces 
  	z_surfaces=[]
  	@objetos.each do |obj|
		objeto2d=proj3d(obj.pontos)
	  	obj.faces.each{|face|
		    z=face[:pontos].map{|idx| obj.pontos[idx]}.map{|e| Point.sub(e,@cam.optic_center).normal}.reduce(&:+)/face[:pontos].size
		    check=@cam.projection_plane.check(obj.pontos[face[:pontos][0]])
		    if true #z_surfaces.include?(check)
			    z_surfaces<<check
			    not_back = face[:pontos].map{|idx| obj.pontos[idx]}.inject(true){|acc,el|
			     	acc &&  @cam.projection_plane.check(el)>0 #(Point.multVetScalar(Point.sub(el,@cam.optic_center),Point.sub(@camPos,@cam.optic_center))>0)
			    }
			    brilho=Point.multVetScalar(Plane.new(*(face[:pontos][0..2].map{|idx|obj.pontos[idx]})).normal.unitario,Point.new(1.8,0.4,-0.8).unitario).abs*220
			    brilho=[[brilho,255].min,0].max
			    brilho=[brilho+35,255].min
			    c=Gosu::Color.rgb(brilho,brilho,brilho)

			    if obj.textura
			    	aux1,aux2,aux3,aux4=face[:pontos].map{|idx| objeto2d[idx]}.map{|e| Point.mult(e,width/(@cam.sizex))}
			    	if not_back and (0.05..120).include?(z)
			    		if obj.texturaunica
							obj.textura.draw_as_quad(aux1.x,aux1.y,c,aux2.x,aux2.y,c,aux3.x,aux3.y,c,aux4.x,aux4.y,c,-z) 
						else
							obj.textura[face[:textura]].draw_as_quad(aux1.x,aux1.y,c,aux2.x,aux2.y,c,aux3.x,aux3.y,c,aux4.x,aux4.y,c,-z) 
						end
					end
			    else
			  		triangulos=[face[:pontos][0..2]]
			  		if face[:pontos].size == 4
			  			triangulos<<([face[:pontos][2..3],face[:pontos][0]].flatten)
			  		end
				    triangulos.each{|tri|
				    	aux1,aux2,aux3=tri.map{|idx| objeto2d[idx]}.map{|e| Point.mult(e,width/(@cam.sizex))}
						draw_triangle(aux1.x,aux1.y,c,aux2.x,aux2.y,c,aux3.x,aux3.y,c,-z) if not_back and (0..120).include?(z) 
					}
				end
			end
		}
	end
  end
  def proj3d objeto
  	if @cam !=nil
		objeto.map{|e|@cam.projXY(e)}
	end
  end
  def needs_cursor?
  	false
  	#true
  end
  def button_down id
  	exit if id == Gosu::KbQ
  	@render_mode = (@render_mode+1) % 4 if id == Gosu::KbM
  end
  #def no_colide pos, z
  #	(@objetos.select{|obj| (obj.posicao.z-z).abs<0.1}.select{|obj|
  #		((obj.posicao.x-obj.raio*1.1)..(obj.posicao.x+obj.raio*1.1)).include?(pos.x) and ((obj.posicao.y-obj.raio*1.1)..(obj.posicao.y+obj.raio*1.1)).include?(pos.y) } == [])
  #end
  def update
  	if button_down?(Gosu::KbW)
  		aux = Point.add(@camPos,Point.mult(@camOrientacao.unitario,-1/8.0))
  		@camPos = aux #if no_colide aux, 1
  	elsif button_down?(Gosu::KbS)
  		aux = Point.add(@camPos,Point.mult(@camOrientacao.unitario, 1/8.0))
  		@camPos = aux #if no_colide aux, 1
  	end
  	if button_down?(Gosu::KbA)
  		aux = Point.add(@camPos,Point.mult(@cam.inferior_edge.unitario, -1/8.0))
  		@camPos = aux #if no_colide aux, 1
  	elsif button_down?(Gosu::KbD)
  		aux = Point.add(@camPos,Point.mult(@cam.inferior_edge.unitario, 1/8.0))
  		@camPos = aux #if no_colide aux, 1
  	end
  	if button_down?(Gosu::KbH)
  		@fov+=0.01
  	elsif button_down?(Gosu::KbJ)
  		@fov-=0.01
  	end
  	if button_down?(Gosu::KbK)
  		@camAperture+=0.03
  	elsif button_down?(Gosu::KbL)
  		@camAperture-=0.03
  	end
  	#@objetos[0].move_to(@camPos)
  	#@objetos.each{|obj| obj.rotaciona(0.01, 0.01, 0.01)}

  	@camOrientacao = Point.spherical(fov_distance,(1-(mouse_y-1)/height)*PI,(mouse_x/width)*PI*2)
  	#@camOrientacao = Point.spherical(fov_distance,PI/2,(mouse_x/width)*PI*2)
  	@cam = Camera::construct(@camPos,@camOrientacao,@camAperture*(width.to_f/height),@camAperture)
  end
end
ProjWindow.new(1920,1080,true).show






