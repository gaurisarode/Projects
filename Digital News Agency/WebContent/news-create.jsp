<%@ include file="header.jsp" %>
<%@ include file="sidebar-user.jsp" %>
<%@ page import = "java.util.Date" %>
      <%@ page import = "java.text.*" %>
 <section id="main-content">
   <section class="wrapper">
   <div class="alert alert-info fade in">
                                  <button data-dismiss="alert" class="close close-sm" type="button">
                                      <i class="icon-remove"></i>
                                  </button>
                                  Welcome <strong><%=session.getAttribute("email") %></strong> Again!! :)
                              </div>
                              <sql:query dataSource="${ds}" var="result">
SELECT * from category;
</sql:query>   
      <div class="row">
                  <div class="col-lg-6">
                      <section class="panel">
                          <header class="panel-heading">
                             <strong>Add News </strong>                 </header>
                          <div class="panel-body">
                          <font color="red" ><strong><c:out value="${msg }"></c:out></strong></font>
                              <form role="form" method="post" action="<%=request.getContextPath() %>/Admin">
                                <input type="hidden" name="email" value="<c:out value="${email}"></c:out>"/>
								<input type="hidden" name="action" value="news-create1"/>
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-1">Date <span class="required">*</span></label>
                                          <select name="date" class="form-control  m-bot13"> 
											<%
											Date d1 = (Date)request.getAttribute("date");
											%>
											<option value="<%=DateFormat.getDateInstance().format(new Date()) %>"> <%=DateFormat.getDateInstance().format(new Date()) %></option>
											<option value="<%=DateFormat.getDateInstance().format(d1) %>"><%=DateFormat.getDateInstance().format(d1) %></option>
											
											</select>
											                                      </div>
                                  
                                  <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Category <span class="required">*</span></label>
                                          <select name="category" class="form-control  m-bot13">
											<c:forEach var="row" items="${result.rows}">
											<option value="<c:out value="${row.id}"/>"><c:out value="${row.name}"/></option>
											</c:forEach>
											</select>
                                      </div>
                                   <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Title <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="title"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Price <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="price"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Short Description <span class="required">*</span></label>
                                           <div class="col-lg-10">
                                              <textarea class="form-control " id="ccomment" name="short-description" required></textarea>
                                          </div>
                                      </div>
                                      
                                       <div class="form-group ">
                                          <label for="cname" class="control-label col-lg-2">Credits <span class="required">*</span></label>
                                          <div class="col-lg-5">
                                              <input class="form-control" id="cname" name="credits"  type="text" required />
                                          </div>
                                      </div>
                                      
                                       <div class="col-lg-12">
                              <section class="panel">
                                       <header class="panel-heading">
                                      News Description                                  </header>                            
                                  <div class="panel-body">
                                      
                                              <div class="form-group">
                                                 
                                                  <div class="col-sm-10">
                                                      <textarea class="form-control ckeditor" name="description" rows="6"></textarea>
                                                  </div>
                                              </div>
                                        
                                      </div>
                                      </section>
                                  </div>
                                  <div class="form-group">
                                          <div class="col-lg-offset-2 col-lg-10">
                                              <button class="btn btn-primary" type="submit">Add News</button>
                                              
                                          </div>
                                      </div>
                                      
                                       </form>
                                       </div>
                              </section>
                          </div>
                                      
                                  
                                
                                  </div>
                   
                      </section>
                  
                  </section>
                  
                  <!-- ck editor -->
    <script type="text/javascript" src="assets/ckeditor/ckeditor.js"></script>
                  <%@ include file="footer.jsp" %>