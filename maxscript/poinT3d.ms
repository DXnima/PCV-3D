-- 删除所有对象
delete objects

-- 获取obj对象中的所有面元素
-- obj 对象
fn getElements obj =
(
	try
	(
		f = obj.numfaces
		eles = #( )
		done = #( )
		for i = 1 to f do
		(
			if ( finditem done i ) == 0 then
			(
				case ( classof obj ) of
				(
					editable_mesh:( faces = ( meshop.getElementsUsingFace obj #( i ) ) as array )
					editable_poly:( faces = ( polyop.getElementsUsingFace obj #( i ) ) as array )
				)
				append eles faces
				join done faces
			)
		)
	)
	catch (
		eles = undefined
	)
	return eles
)

-- 指定对象指定面分离为单独元素
-- obj 对象
-- ele 元素
-- newName 新对象名称
fn DetachElement obj ele newName =
(
	polyOp.detachFaces obj ele asNode:true name:newName
	detachedObj = ( getNodeByName newName )
	centerPivot detachedObj --没有这句话的话，新的物体的轴心还是在原来的物体的轴心的位置
	return detachedObj
)

-- 批量分离对象
-- obj 对象
fn DetachElements obj = 
(
	eleList = getElements obj
	index = 0
	for ele in eleList[1] do
	(
		m = mod ele 6
		if m == 1 then (
			newName = ( 5 + index * 6 ) as string
		)
		if m == 2 then (
			newName = ( 6 + index * 6 ) as string
		)
		if m == 3 then (
			newName = ( 1 + index * 6 ) as string
		)
		if m == 4 then (
			newName = ( 2 + index * 6 ) as string
		)
		if m == 5 then (
			newName = ( 3 + index * 6 ) as string
		)
		if m == 0 then (
			newName = ( 4 + index * 6 ) as string
			index += 1
		)
		DetachElement obj eleList[1][1] newName
	)
	-- 删除
	delete obj
)

-- 挤出模型的根据数目分段
-- sline 对象
-- d 挤出数
-- i边
-- j边
fn clipPlane sline d i j = (
	select sline
	p = convertTo sline ( Editable_Poly ) -- Editable_Poly 类实例
	subobjectLevel = 2
	p.SetSelection #Edge #{ i, j } -- 选择边
	p.connectEdgeSegments = ( d ) as integer -- 设置连接数
	p.ConnectEdges()
	redrawViews() -- 视图刷新
)

-- 位图材质贴图
-- size 管道节数
fn bitTexture size = (
	file = getSavePath caption:"请选取纹理图片文件夹" types: "*.*"
	-- filePath = getFilenamePath ( file )
	for i = 1 to size do (
		s = file + "\\" + ( i ) as string + ".png" -- 图片路径
		meditmaterials[i].diffuseMap = bitmaptexture filename:s -- 图片赋值给材质球
		select objects[i] -- 选中对象
		$.material = meditmaterials[i] -- 对应面片赋值材质
		showTextureMap meditmaterials[j] on -- 开启视口中显示明暗处理材质
	)
	-- rotate p -90 y_axis -- 绕y轴旋转-90°
	redrawViews()
)

-- uvw贴图
-- size 对象个数
fn uvwMapTexture size = (
	for i = 1 to size do (
		select objects[i] -- 选择指定对象
		addModifier objects[i] ( UVWMap()) -- 添加uvw修改器
		uvw = objects[i].Modifiers[#UVW_Map] -- 获取uvw修改器
		uvw.maptype = 4 -- 设置长方体贴图
		uvw.height = 2 -- 设置高度
		-- 如果是平面1和5
		if i == 1 or i == 5 then (
			uvw.length = 0.1 -- 设置长
			uvw.width = 2.5 -- 设置宽度 
		)
		-- 如果是平面2和4
		if i == 2 or i == 4 then (
			uvw.length = 0.5 -- 设置长
			uvw.width = 1 -- 设置宽度
		)
		-- 如果是平面3
		if i == 3 then (
			uvw.length = 4 -- 设置长
			uvw.width = 0.1 -- 设置宽度
		)
		-- 如果是平面6
		if i == 6 then (
			uvw.length = 5 -- 设置长
			uvw.width = 0.1 -- 设置宽度
		)
	)
)

-- 创建面模型
-- arr 点数组
fn creatPlane arr = (
	count = arr.count
	for i = 3 to count by 4 do (
		sline = SplineShape pos:arr[2] prefix:"plane"
		addNewSpline sline -- 创建一条线
		addKnot sline 1 #corner #line arr[i + 0] -- 依次连接
		addKnot sline 1 #corner #line arr[i + 1]
		addKnot sline 1 #corner #line arr[i + 2]
		addKnot sline 1 #corner #line arr[i + 3]
		close sline 1 -- 线闭合
		convertToPoly sline -- 线条转面
		-- rotate sline 90 y_axis -- 绕y轴旋转90°
	)
	redrawViews() -- 视图刷新
)

-- 挤出模型的根据数目分段
-- sline 对象
-- d 分段数
fn clipPlaneExtr sline d = (
	select sline
	p = convertTo sline ( Editable_Poly ) -- Editable_Poly 类实例
	subobjectLevel = 2
	p.SetSelection #Edge #{ 13..18 } -- 选择边
	p.connectEdgeSegments = ( d ) as integer -- 设置连接数
	p.ConnectEdges()
	redrawViews() -- 视图刷新
)

-- 挤出创建面模型
-- arr 点数组
-- size 挤出面的节数
fn creatPlaneExtr arr size = (
	sline = SplineShape name:"plane001" pos:arr[2] 
	addNewSpline sline -- 创建一条线
	for i = 3 to 8 do (
		addKnot sline 1 #corner #line arr[i] -- 依次连接
	)
	close sline 1 -- 线闭合
	p = convertToPoly sline -- 线条转面
	for i = 1 to size do (
		-- 循环挤出
		polyop.extrudeFaces p #{ 1 } 2 -- 挤出
	)
	polyOp.deleteFaces p #{ 1 } delIsoVerts:false -- 删除顶面
	-- rotate p -90 y_axis -- 绕y轴旋转90°
	redrawViews() -- 视图刷新
)

-- 材质和贴图整合
-- size 节数
fn textureExtr size = (
	file = getSavePath caption:"请选取纹理图片文件夹" types: "*.*"
	index = 1
	for i = 1 to size do (
		for j = 1 to 6 do (
			s = file + "\\" + ( i ) as string + "_" + ( j ) as string + ".jpg" -- 图片路径
			/* 对象赋值材质 */
			meditmaterials[j].diffuseMap = bitmaptexture filename:s -- 图片赋值给材质球
			obj = getNodeByName ( index as string )
			select obj -- 选中对象
			$.material = meditmaterials[j] -- 对应面片赋值材质
			showTextureMap meditmaterials[j] on -- 开启视口中显示明暗处理材质
			/* 开始uvw贴图 */
			m = mod index 6 -- 计算6的余数
			addModifier obj ( UVWMap()) -- 添加uvw修改器
			uvw = obj.Modifiers[#UVW_Map] -- 获取uvw修改器
			uvw.maptype = 4 -- 设置长方体贴图
			uvw.height = 2 -- 设置高度
			-- 如果是左平面1
			if m == 1 then (
				uvw.length = 0.1 -- 设置长
				uvw.width = 2 -- 设置宽度 
				uvw.height = 2.35 -- 设置高度
				uvw.uflip = on
				-- y轴旋转90°
				pos = uvw.Gizmo.Position
				uvw.Gizmo.Rotation = ( quat -0.707107 0 0.707107 0 )
				uvw.Gizmo.Position = pos
			)
			-- 如果是右平面5
			if m == 5 then (
				uvw.length = 0.1 -- 设置长
				uvw.width = 2 -- 设置宽度 
				uvw.height = 2.53 -- 设置高度
				uvw.uflip = on
				-- y轴旋转90°
				pos = uvw.Gizmo.Position
				uvw.Gizmo.Rotation = ( quat -0.707107 0 0.707107 0 )
				uvw.Gizmo.Position = pos
			)
			-- 如果是左斜右斜平面
			if m == 2 or m == 4 then (
				uvw.length = 0.45 -- 设置长
				uvw.width = 0.7 -- 设置宽度
			)
			-- 如果是上平面
			if m == 3 then (
				uvw.length = 4.1 -- 设置长
				uvw.width = 0.1 -- 设置宽度
			)
			-- 如果是下平面
			if m == 0 then (
				uvw.length = 4.8 -- 设置长
				uvw.width = 0.1 -- 设置宽度
			)
			index += 1
			-- 刷新
			redrawViews()
		)
		-- 重置材质编辑器
		macros.run "Medit Tools" "clear_medit_slots"
	)
	-- rotate p -90 y_axis -- 绕y轴旋转-90°
)

undo on (
	global arr = #( )
	-- f = getOpenFileName types: "txt(*.txt)" -- 打开文件对话框
	-- fs = openfile f -- 读取文件
	fs = openfile "planeinter1.txt"
	while ( ( eof fs ) != true ) do (
		strLine = readLine fs
		arrLine = FilterString strLine " "
		x = ( arrLine[1] ) as float
		y = ( arrLine[2] ) as float
		z = ( arrLine[3] ) as float
		local p = [x, y, z]
		append arr p
	)
	-- 关闭文件
	close fs
	-- 挤出创建面模型的节数
	size = 40
	-- 挤出创建面模型
	creatPlaneExtr arr size
	-- 分离对象
	DetachElements $plane001
	-- 模型材质贴图
	textureExtr size
)